import cv2
import numpy as np
from funciones_auxiliares import linea_discontinua 
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import threading
import time
class Camara(Node):
    
    def __init__(self, archivo_valores="valores_lower_upper_refpoint.txt"):
        #iniciar ros
        super().__init__('camara')
        self.publisher_ = self.create_publisher(String, 'target_reference', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # publish every second
        self.get_logger().info("Camara node started, publishing to 'target_reference'")
        
        #iniciar camara
        self.x = 0
        self.y = 0
        self.running = True
        # Cargar parámetros desde archivo
        self.lower, self.upper, self.punto_de_referencia, self.color1_hsv, self.ksize = self.cargar_valores_desde_archivo(archivo_valores)
        self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        self.thread = threading.Thread(target=self.ejecutar, daemon=True)
        self.thread.start()
        
    def cargar_valores_desde_archivo(self, filename):
        with open(filename, "r") as f:
            lines = f.readlines()
            lower = np.array(eval(lines[0].split(":")[1].strip()))
            upper = np.array(eval(lines[1].split(":")[1].strip()))
            punto_de_referencia = np.array(eval(lines[2].split(":")[1].strip()))
            color1_hsv = np.array(eval(lines[3].split(":")[1].strip()))
            ksize = int(eval(lines[4].split(":")[1].strip()))
        return lower, upper, punto_de_referencia, color1_hsv, ksize
    
    def timer_callback(self):
        # Example tuple: (x, y)
        target = (self.x, self.y)
        msg = String()
        msg.data = str(target)  # publish as string for now
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

    def ejecutar(self):
        try:
            while self.running:
                ret, frame = self.cap.read()
                if not ret:
                    continue

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