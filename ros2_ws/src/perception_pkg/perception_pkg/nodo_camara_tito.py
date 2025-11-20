#!/usr/bin/env python3
import threading
import time
import sys

import numpy as np
import cv2
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

class Camara(Node):
    def __init__(self,
                 archivo_valores: str = "valores_lower_upper_refpoint.txt"):
        super().__init__('camara')

        # ROS parameters: allow toggling GUI at runtime via ros2 param
        self.declare_parameter("use_gui", False)
        self.declare_parameter("archivo_valores", archivo_valores)

        self.use_gui = bool(self.get_parameter("use_gui").get_parameter_value().bool_value)
        archivo_valores = self.get_parameter("archivo_valores").get_parameter_value().string_value

        # Publisher
        self.publisher_ = self.create_publisher(String, 'target_reference', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # publish every second
        self.get_logger().info("Camara node started, publishing to 'target_reference'")

        # State
        self.x = 0
        self.y = 0
        self.running = True
        self.lock = threading.Lock()  # protect centroid and other shared vars

        # Load parameters from file (or use safe defaults)
        try:
            self.lower, self.upper, self.punto_de_referencia, self.color1_hsv, self.ksize = \
                self.cargar_valores_desde_archivo(archivo_valores)
            # ensure types
            self.lower = np.array(self.lower, dtype=np.int32)
            self.upper = np.array(self.upper, dtype=np.int32)
            self.punto_de_referencia = list(self.punto_de_referencia)
            self.color1_hsv = np.array(self.color1_hsv, dtype=np.int32)
            self.ksize = int(self.ksize)
            self.get_logger().info(f"Loaded values from {archivo_valores}")
        except Exception as e:
            self.get_logger().warn(f"Could not load {archivo_valores}, using defaults: {e}")
            self.lower = np.array([0, 70, 50])
            self.upper = np.array([179, 255, 255])
            self.punto_de_referencia = [320, 240]
            self.color1_hsv = np.array([0, 0, 0])
            self.ksize = 1

        # RealSense pipeline + align (start once)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        color_w, color_h, fps = 640, 480, 30
        depth_w, depth_h = 640, 480
        self.config.enable_stream(rs.stream.color, color_w, color_h, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, depth_w, depth_h, rs.format.z16, fps)
        # (Optional IMU for D435i: self.config.enable_stream(rs.stream.gyro); self.config.enable_stream(rs.stream.accel))

        try:
            self.profile = self.pipeline.start(self.config)
            self.align = rs.align(rs.stream.color)
            self.get_logger().info("RealSense pipeline started successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense pipeline: {e}")
            raise

        # GUI setup if requested
        if self.use_gui:
            cv2.namedWindow("image")
            cv2.resizeWindow("image", 540, 480)
            cv2.moveWindow("image", 700, 100)

            cv2.createTrackbar("HMin", "image", int(self.lower[0]), 179, self._nothing)
            cv2.createTrackbar("SMin", "image", int(self.lower[1]), 255, self._nothing)
            cv2.createTrackbar("VMin", "image", int(self.lower[2]), 255, self._nothing)
            cv2.createTrackbar("HMax", "image", int(self.upper[0]), 179, self._nothing)
            cv2.createTrackbar("SMax", "image", int(self.upper[1]), 255, self._nothing)
            cv2.createTrackbar("VMax", "image", int(self.upper[2]), 255, self._nothing)
            cv2.createTrackbar("ksize", "image", max(1, int(self.ksize)), 51, self._nothing)

            cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("frame", color_w, color_h)
            cv2.moveWindow("frame", 30, 100)
            cv2.setMouseCallback("frame", self._mouseEvent)

            # if you have an hsv_color_map.png, show it; if not create a placeholder
            try:
                self.image_ref = cv2.imread("hsv_color_map.png")
                if self.image_ref is None:
                    raise FileNotFoundError
            except Exception:
                self.image_ref = np.zeros((480, 540, 3), dtype=np.uint8)

        # Thread to capture frames and process
        self.thread = threading.Thread(target=self.ejecutar, daemon=True)
        self.thread.start()

        # Ensure we save values filename later
        self.archivo_valores = archivo_valores

    # ----------------------
    # Utilities
    # ----------------------
    def _nothing(self, x):
        pass

    def cargar_valores_desde_archivo(self, filename):
        with open(filename, "r") as f:
            lines = f.readlines()
            lower = np.array(eval(lines[0].split(":")[1].strip()))
            upper = np.array(eval(lines[1].split(":")[1].strip()))
            punto_de_referencia = np.array(eval(lines[2].split(":")[1].strip()))
            color1_hsv = np.array(eval(lines[3].split(":")[1].strip()))
            ksize = int(eval(lines[4].split(":")[1].strip()))
        return lower, upper, punto_de_referencia, color1_hsv, ksize

    def _mouseEvent(self, event, x, y, flags, param):
        # Left click: pick HSV from current frame
        if event == cv2.EVENT_LBUTTONDOWN:
            if hasattr(self, "frame") and self.frame is not None:
                hsv_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
                picked = hsv_frame[y, x]
                self.color1_hsv = picked
                # update trackbars (with safety)
                cv2.setTrackbarPos("HMin", "image", max(int(self.color1_hsv[0] - 40), 0))
                cv2.setTrackbarPos("SMin", "image", max(int(self.color1_hsv[1] - 45), 0))
                cv2.setTrackbarPos("VMin", "image", max(int(self.color1_hsv[2] - 75), 0))
                cv2.setTrackbarPos("HMax", "image", min(int(self.color1_hsv[0] + 40), 179))
                cv2.setTrackbarPos("SMax", "image", min(int(self.color1_hsv[1] + 45), 255))
                cv2.setTrackbarPos("VMax", "image", 255)
                self.get_logger().info(f"Picked HSV: {self.color1_hsv.tolist()}")
        # Right click: set reference point
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.punto_de_referencia = [x, y]
            self.get_logger().info(f"Reference point set to: {self.punto_de_referencia}")

    # ----------------------
    # ROS timer callback: publishes (x,y,distance_m) as string
    # ----------------------
    def timer_callback(self):
        with self.lock:
            x, y = int(self.x), int(self.y)
        # attempt to read distance if possible
        dist = None
        if hasattr(self, "last_depth_frame") and self.last_depth_frame is not None:
            try:
                # bounds-check x,y
                h = self.last_depth_frame.get_height()
                w = self.last_depth_frame.get_width()
                px = max(0, min(w - 1, x))
                py = max(0, min(h - 1, y))
                dist = self.last_depth_frame.get_distance(px, py)
            except Exception as e:
                self.get_logger().debug(f"Could not get distance: {e}")
                dist = None

        msg = String()
        if dist is None:
            msg.data = str((x, y, None))
        else:
            msg.data = str((x, y, float(dist)))
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Publishing: {msg.data}')

    # ----------------------
    # Main loop (runs in background thread)
    # ----------------------
    def ejecutar(self):
        try:
            while self.running:
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                except Exception as e:
                    self.get_logger().warn(f"Timeout or error getting frames: {e}")
                    continue

                aligned = self.align.process(frames)
                color_frame = aligned.get_color_frame()
                depth_frame = aligned.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                # convert color frame to numpy (BGR)
                color_image = np.asanyarray(color_frame.get_data())

                # store latest depth_frame for timer callback
                self.last_depth_frame = depth_frame

                # get trackbar values if GUI is enabled
                if self.use_gui:
                    hMin = cv2.getTrackbarPos("HMin", "image")
                    sMin = cv2.getTrackbarPos("SMin", "image")
                    vMin = cv2.getTrackbarPos("VMin", "image")
                    hMax = cv2.getTrackbarPos("HMax", "image")
                    sMax = cv2.getTrackbarPos("SMax", "image")
                    vMax = cv2.getTrackbarPos("VMax", "image")
                    ksize = cv2.getTrackbarPos("ksize", "image")
                else:
                    hMin, sMin, vMin = int(self.lower[0]), int(self.lower[1]), int(self.lower[2])
                    hMax, sMax, vMax = int(self.upper[0]), int(self.upper[1]), int(self.upper[2])
                    ksize = int(self.ksize)

                # safety for kernel size
                if ksize < 1:
                    ksize = 1
                if ksize % 2 == 0:
                    k = ksize + 1
                else:
                    k = ksize

                lower = np.array([hMin, sMin, vMin])
                upper = np.array([hMax, sMax, vMax])

                # apply blur if needed
                if k > 1:
                    proc = cv2.blur(color_image, (k, k))
                else:
                    proc = color_image.copy()

                # mask + contours
                frameHSV = cv2.cvtColor(proc, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(frameHSV, lower, upper)
                contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # ensure we have a reference point
                if not hasattr(self, "punto_de_referencia") or self.punto_de_referencia is None:
                    self.punto_de_referencia = [int(color_image.shape[1] / 2), int(color_image.shape[0] / 2)]

                Contornos_Pos = []
                for c in contornos:
                    area = cv2.contourArea(c)
                    if area > 1000:
                        M = cv2.moments(c)
                        if M.get("m00", 0) == 0:
                            continue
                        x = int(M["m10"] / M["m00"])
                        y = int(M["m01"] / M["m00"])
                        distancia = np.sqrt((x - self.punto_de_referencia[0])**2 + (y - self.punto_de_referencia[1])**2)
                        Contornos_Pos.append([c, distancia, (x, y)])
                        # draw candidate for GUI
                        if self.use_gui:
                            cv2.circle(proc, (x, y), 4, (255, 0, 255), -1)

                if Contornos_Pos:
                    Contornos_Pos.sort(key=lambda x: x[1])
                    _, _, centroide = Contornos_Pos[0]
                    with self.lock:
                        self.x, self.y = centroide
                else:
                    # nothing found: keep previous or set to reference
                    pass

                # draw reference point & line & text if GUI
                if self.use_gui:
                    # draw reference point
                    cv2.circle(proc, tuple(self.punto_de_referencia), 6, (239, 255, 0), -1)
                    cv2.putText(proc, "punto de referencia",
                                (self.punto_de_referencia[0] + 10, self.punto_de_referencia[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (239, 255, 0), 1, cv2.LINE_AA)
                    # draw line to centroid if exists
                    with self.lock:
                        cx, cy = int(self.x), int(self.y)
                    if cx != 0 or cy != 0:
                        cv2.line(proc, tuple(self.punto_de_referencia), (cx, cy), (0, 255, 255), 2)
                        cv2.putText(proc, f"Centroide: ({cx},{cy})", (cx + 10, cy - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

                    # show mask-ref
                    mask_ref = cv2.bitwise_and(self.image_ref, self.image_ref, mask=cv2.inRange(cv2.cvtColor(self.image_ref, cv2.COLOR_BGR2HSV), lower, upper))
                    cv2.imshow("image", mask_ref)
                    cv2.imshow("frame", proc)

                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        self.get_logger().info("Quit pressed in GUI, stopping.")
                        self.running = False
                        break

                # small sleep to avoid hogging CPU
                time.sleep(0.002)

        except Exception as e:
            self.get_logger().error(f"Exception in ejecutar(): {e}")
        finally:
            # cleanup (pipeline stop happens in destroy)
            pass

    # ----------------------
    # Clean shutdown
    # ----------------------
    def stop(self):
        self.running = False
        # wait for thread finish
        try:
            if self.thread.is_alive():
                self.thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            self.pipeline.stop()
        except Exception:
            pass
        if self.use_gui:
            cv2.destroyAllWindows()
        # save values
        try:
            hMin = cv2.getTrackbarPos("HMin", "image") if self.use_gui else int(self.lower[0])
            sMin = cv2.getTrackbarPos("SMin", "image") if self.use_gui else int(self.lower[1])
            vMin = cv2.getTrackbarPos("VMin", "image") if self.use_gui else int(self.lower[2])
            hMax = cv2.getTrackbarPos("HMax", "image") if self.use_gui else int(self.upper[0])
            sMax = cv2.getTrackbarPos("SMax", "image") if self.use_gui else int(self.upper[1])
            vMax = cv2.getTrackbarPos("VMax", "image") if self.use_gui else int(self.upper[2])
            ksize = cv2.getTrackbarPos("ksize", "image") if self.use_gui else int(self.ksize)
        except Exception:
            # fallback to stored values
            hMin, sMin, vMin = int(self.lower[0]), int(self.lower[1]), int(self.lower[2])
            hMax, sMax, vMax = int(self.upper[0]), int(self.upper[1]), int(self.upper[2])
            ksize = int(self.ksize)

        try:
            with open(self.archivo_valores, "w") as f:
                f.write(f"lower: {[hMin, sMin, vMin]}\n")
                f.write(f"upper: {[hMax, sMax, vMax]}\n")
                f.write(f"punto_de_referencia: {[int(self.punto_de_referencia[0]), int(self.punto_de_referencia[1])]}\n")
                f.write(f"color_a_rastrear: {self.color1_hsv.tolist()}\n")
                f.write(f"ksize: {ksize}\n")
            self.get_logger().info(f"Saved values to {self.archivo_valores}")
        except Exception as e:
            self.get_logger().warn(f"Could not save values: {e}")

# ----------------------
# Main
# ----------------------
def main(args=None):
    rclpy.init(args=args)
    node = Camara()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
