import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image
import threading
import time
import depthai as dai
import math

class Camara(Node):
    
    def __init__(self, archivo_valores="valores_lower_upper_refpoint.txt"):
        #iniciar ros
        super().__init__('camara')
        self.publisher_ = self.create_publisher(String, 'target_reference', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # publish every second
        self.get_logger().info("Camara node started, publishing to 'target_reference'")
        self.image_rviz = self.create_publisher(Image, 'camera/image', 10)
        #iniciar camara
        self.x = 0
        self.y = 0
        self.running = True
        # Cargar parámetros desde archivo
        self.lower, self.upper, self.punto_de_referencia, self.color1_hsv, self.ksize = self.cargar_valores_desde_archivo(archivo_valores)
        pipeline = dai.Pipeline()

        cam = pipeline.createColorCamera()
        cam.setPreviewSize(640, 480)
        cam.setInterleaved(False)

        xout = pipeline.createXLinkOut()
        xout.setStreamName("cam_out")
        cam.preview.link(xout.input)

        self.device = dai.Device(pipeline)
        self.q = self.device.getOutputQueue("cam_out", 4, False)

        self.thread = threading.Thread(target=self.ejecutar, daemon=True)
        self.thread.start()
        
    def cargar_valores_desde_archivo(self, filename):
        from ament_index_python.packages import get_package_share_directory
        import os

        pkg_share = get_package_share_directory('perception_pkg')
        filepath = os.path.join(pkg_share, filename)

        with open(filepath, "r") as f:
            lines = f.readlines()
            lower = np.array(eval(lines[0].split(":")[1].strip()))
            upper = np.array(eval(lines[1].split(":")[1].strip()))
            punto_de_referencia = np.array(eval(lines[2].split(":")[1].strip()))
            color1_hsv = np.array(eval(lines[3].split(":")[1].strip()))
            ksize = int(eval(lines[4].split(":")[1].strip()))

        return lower, upper, punto_de_referencia, color1_hsv, ksize

    
    def timer_callback(self):
        # Example tuple: (x, y)
        refx = self.punto_de_referencia[0]
        refy = self.punto_de_referencia[1]
        dx = refx - self.x 
        dy = refy - self.y
        dist = math.sqrt(dx**2 + dy**2)
        angulo_rad = math.atan2(dy, dx)
        angulo_deg = 90 - math.degrees(angulo_rad)
        if angulo_deg > 180:
            angulo_deg -= 360
        elif angulo_deg <= -180:
            angulo_deg += 360
        target = (refx, refy, self.x, self.y, dist, angulo_deg)
        msg = String()
        msg.data = str(target)  # publish as string for now
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

    def ejecutar(self):
        try:
            while self.running:
                in_frame = self.q.get()
                frame = in_frame.getCvFrame()

                # enviar imagen a rviz
                img_msg = Image()
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.height = frame.shape[0]
                img_msg.width = frame.shape[1]
                img_msg.encoding = 'bgr8'
                img_msg.is_bigendian = False
                img_msg.step = frame.shape[1] * 3
                img_msg.data = frame.tobytes()
                self.image_rviz.publish(img_msg)

                # Aplicar blur si ksize > 1
                if self.ksize > 1:
                    frame = cv2.blur(frame, (self.ksize, self.ksize))

                frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(frameHSV, self.lower, self.upper)
                contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                centroide_detectado = False
                Contornos_Pos = []
                for c in contornos:
                    area = cv2.contourArea(c)
                    if area > 1000:
                        M = cv2.moments(c)
                        if M["m00"] == 0:
                            M["m00"] = 1
                        x = int(M["m10"] / M["m00"])
                        y = int(M["m01"] / M["m00"])
                        distancia = np.sqrt((x - self.punto_de_referencia[0])**2 + (y - self.punto_de_referencia[1])**2)
                        Contornos_Pos.append([c, distancia, (x, y)])

                if Contornos_Pos:
                    Contornos_Pos.sort(key=lambda x: x[1])
                    _, _, centroide = Contornos_Pos[0]
                    self.x, self.y = centroide
                    centroide_detectado = True

        except KeyboardInterrupt:
            pass
        finally:
            self.cap.release()
            print("Programa terminado.")

def main(args=None):
    rclpy.init(args=args)
    node = Camara()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.running = False
        node.thread.join()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()