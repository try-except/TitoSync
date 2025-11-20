#!/usr/bin/env python3
import pyrealsense2 as rs
import cv2
import numpy as np
from funciones_auxiliares import linea_discontinua, grados_entre_vectores, mascara_circulo
import time
import threading

class Centroide():
    def __init__(self):
        # --- Valores por defecto (se sobrescriben desde archivo si existe) ---
        self.x = 0
        self.y = 0
        self.LowerColorError = np.array([-40, -45, -75])
        self.UpperColorError = np.array([40, 45, 75])
        self.lock = threading.Lock()
        # cargar valores desde archivo (si no existe, la función debe manejar excepción)
        try:
            self.lower_init, self.upper_init, self.punto_de_referencia, self.color1_hsv, self.ksize_init = \
                self.cargar_valores_desde_archivo("valores_lower_upper_refpoint.txt")
        except Exception as e:
            print("No se pudo leer valores desde archivo, usando defaults:", e)
            self.lower_init = np.array([0, 70, 50])
            self.upper_init = np.array([179, 255, 255])
            self.punto_de_referencia = [320, 240]  # centro como defecto
            self.color1_hsv = np.array([0, 0, 0])
            self.ksize_init = 1

        # --- Ventana de calibración ---
        self.image = cv2.imread("hsv_color_map.png")
        if self.image is None:
            # si no hay imagen de referencia, crea una placeholder para no romper UI
            self.image = np.zeros((480, 540, 3), dtype=np.uint8)
        cv2.namedWindow("image")
        cv2.resizeWindow("image", 540, 480)
        cv2.moveWindow("image", 700, 100)

        cv2.createTrackbar("HMin", "image", 0, 179, self.nothing)
        cv2.createTrackbar("SMin", "image", 0, 255, self.nothing)
        cv2.createTrackbar("VMin", "image", 0, 255, self.nothing)
        cv2.createTrackbar("HMax", "image", 0, 179, self.nothing)
        cv2.createTrackbar("SMax", "image", 0, 255, self.nothing)
        cv2.createTrackbar("VMax", "image", 0, 255, self.nothing)
        cv2.createTrackbar("ksize", "image", 1, 50, self.nothing)

        cv2.setTrackbarPos("HMin", "image", int(self.lower_init[0]))
        cv2.setTrackbarPos("SMin", "image", int(self.lower_init[1]))
        cv2.setTrackbarPos("VMin", "image", int(self.lower_init[2]))
        cv2.setTrackbarPos("HMax", "image", int(self.upper_init[0]))
        cv2.setTrackbarPos("SMax", "image", int(self.upper_init[1]))
        cv2.setTrackbarPos("VMax", "image", int(self.upper_init[2]))
        cv2.setTrackbarPos("ksize", "image", int(self.ksize_init))

        # --- Ventana de frame ---
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 640, 480)
        cv2.moveWindow("frame", 30, 100)
        cv2.setMouseCallback("frame", self._mouseEvent)

        # --- Iniciar RealSense pipeline (una sola vez) ---
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # Puedes cambiar resolución y fps aquí si lo deseas
        color_w, color_h, fps = 640, 480, 30
        depth_w, depth_h = 640, 480
        self.config.enable_stream(rs.stream.color, color_w, color_h, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, depth_w, depth_h, rs.format.z16, fps)
        # Para D435i también podrías activar imu: config.enable_stream(rs.stream.gyro); config.enable_stream(rs.stream.accel)

        # Iniciar pipeline y align (alinear depth a color)
        try:
            self.profile = self.pipeline.start(self.config)
            self.align = rs.align(rs.stream.color)
            print("RealSense pipeline started OK")
        except Exception as e:
            print("Error al iniciar RealSense pipeline:", e)
            raise

        # Ejecutar loop principal
        self.ejecutar()

    # --- Función para cargar valores previos ---
    def cargar_valores_desde_archivo(self, filename):
        with open(filename, "r") as file:
            lines = file.readlines()
            lower = np.array(eval(lines[0].split(":")[1].strip()))
            upper = np.array(eval(lines[1].split(":")[1].strip()))
            reference_point = np.array(eval(lines[2].split(":")[1].strip()))
            color1_hsv = np.array(eval(lines[3].split(":")[1].strip()))
            ksize_init = np.array(eval(lines[4].split(":")[1].strip()))
        return lower, upper, reference_point.tolist(), color1_hsv, int(ksize_init)

    def _mouseEvent(self, event, x, y, flags, param):
        # Left click: pick HSV from current frame
        if event == cv2.EVENT_LBUTTONDOWN:
            # read frame safely
            with self.lock:
                frame_copy = self.frame.copy() if hasattr(self, "frame") and self.frame is not None else None
            if frame_copy is not None:
                # guard against clicks outside image (if window resized)
                h, w = frame_copy.shape[:2]
                px = max(0, min(w - 1, x))
                py = max(0, min(h - 1, y))
                hsv_frame = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2HSV)
                picked = hsv_frame[py, px]
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


    def nothing(self, x):
        pass

    def dibujar_punto_de_ref(self, frame):
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.circle(frame, tuple(self.punto_de_referencia), 7, (239, 255, 0), -1)
        cv2.putText(frame, "punto de referencia",
                    (self.punto_de_referencia[0] + 10, self.punto_de_referencia[1]),
                    font, 0.75, (239, 255, 0), 1, cv2.LINE_AA)
        linea_discontinua(frame, (self.punto_de_referencia[0], 0),
                          (self.punto_de_referencia[0], frame.shape[0]), (0, 239, 255))
        cv2.arrowedLine(frame, tuple(self.punto_de_referencia),
                        (self.punto_de_referencia[0], self.punto_de_referencia[1] + round(frame.shape[0] / 4)),
                        (239, 255, 0), 2, tipLength=0.2)

    def ejecutar(self):
        try:
            while True:
                # Obtener valores HSV actuales desde los trackbars
                hMin = cv2.getTrackbarPos("HMin", "image")
                sMin = cv2.getTrackbarPos("SMin", "image")
                vMin = cv2.getTrackbarPos("VMin", "image")
                hMax = cv2.getTrackbarPos("HMax", "image")
                sMax = cv2.getTrackbarPos("SMax", "image")
                vMax = cv2.getTrackbarPos("VMax", "image")
                ksize = cv2.getTrackbarPos("ksize", "image")
                if ksize < 1:
                    ksize = 1

                lower = np.array([hMin, sMin, vMin])
                upper = np.array([hMax, sMax, vMax])

                # Mostrar la imagen de calibración con el resultado de la máscara (solo visual)
                try:
                    hsv_ref = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
                    mask_ref = cv2.inRange(hsv_ref, lower, upper)
                    result_ref = cv2.bitwise_and(self.image, self.image, mask=mask_ref)
                    cv2.imshow("image", result_ref)
                except Exception:
                    # si image no es válida, no romper el loop
                    pass

                # --- Captura frames from RealSense ---
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                except Exception as e:
                    print("Timeout o error al obtener frames:", e)
                    continue

                # Alinear depth a color
                aligned = self.align.process(frames)
                color_frame = aligned.get_color_frame()
                depth_frame = aligned.get_depth_frame()
                if not color_frame or not depth_frame:
                    # no hay frames válidos
                    continue

                # Convertir color a numpy BGR
                color_image = np.asanyarray(color_frame.get_data())

                # store latest depth_frame for optional distance queries
                self.last_depth_frame = depth_frame

                # keep a thread-safe copy for the mouse callback
                with self.lock:
                    self.frame = color_image.copy()

                # Aplicar blur (usar ksize impar si quieres kernel simétrico)
                k = ksize if ksize % 2 == 1 else ksize + 1
                if k > 1:
                    color_blur = cv2.blur(color_image, (k, k))
                else:
                    color_blur = color_image.copy()

                # Procesamiento HSV y contornos sobre la imagen blur
                frameHSV = cv2.cvtColor(color_blur, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(frameHSV, lower, upper)
                contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # Asegurar punto de referencia
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
                        distancia = ((x - self.punto_de_referencia[0]) ** 2 + (y - self.punto_de_referencia[1]) ** 2) ** 0.5
                        cv2.circle(color_blur, (x, y), 7, (255, 0, 255), -1)
                        Contornos_Pos.append([c, distancia, (x, y)])

                Contornos_Pos.sort(key=lambda x: x[1])

                if Contornos_Pos:
                    _, _, centroide = Contornos_Pos[0]
                    # proteger acceso concurrente
                    with self.lock:
                        self.x, self.y = centroide
                    cv2.line(color_blur, tuple(self.punto_de_referencia), (self.x, self.y), (0, 255, 255), 2)
                    cv2.putText(color_blur, f"Centroide: ({self.x},{self.y})", (self.x + 10, self.y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                    print(f"Centroide detectado en: ({self.x}, {self.y})")

                # Mostrar frame
                cv2.imshow("frame", color_blur)

                # tecla para salir o guardar
                key = cv2.waitKey(1) & 0xFF
                if key == ord("s"):
                    break
                if key == ord("q"):
                    break

        except KeyboardInterrupt:
            pass

        finally:
            # Guardar valores actuales en archivo
            try:
                hMin = cv2.getTrackbarPos("HMin", "image")
                sMin = cv2.getTrackbarPos("SMin", "image")
                vMin = cv2.getTrackbarPos("VMin", "image")
                hMax = cv2.getTrackbarPos("HMax", "image")
                sMax = cv2.getTrackbarPos("SMax", "image")
                vMax = cv2.getTrackbarPos("VMax", "image")
                ksize = cv2.getTrackbarPos("ksize", "image")
            except Exception:
                # fallback a valores leidos
                hMin, sMin, vMin = int(self.lower_init[0]), int(self.lower_init[1]), int(self.lower_init[2])
                hMax, sMax, vMax = int(self.upper_init[0]), int(self.upper_init[1]), int(self.upper_init[2])
                ksize = int(self.ksize_init)

            lower = [hMin, sMin, vMin]
            upper = [hMax, sMax, vMax]
            try:
                with open("valores_lower_upper_refpoint.txt", "w") as f:
                    f.write(f"lower: {lower}\n")
                    f.write(f"upper: {upper}\n")
                    f.write(f"punto_de_referencia: {[int(self.punto_de_referencia[0]), int(self.punto_de_referencia[1])]}\n")
                    f.write(f"color_a_rastrear: {self.color1_hsv.tolist()}\n")
                    f.write(f"ksize: {ksize}\n")
            except Exception as e:
                print("No se pudo escribir archivo:", e)

            # Liberar recursos RealSense y OpenCV
            try:
                self.pipeline.stop()
            except Exception:
                pass
            cv2.destroyAllWindows()



if __name__ == "__main__":
    detector = Centroide()
