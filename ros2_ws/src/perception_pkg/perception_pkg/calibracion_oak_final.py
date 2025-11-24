import cv2
import numpy as np
from funciones_auxiliares import linea_discontinua, grados_entre_vectores, mascara_circulo
import threading
from pathlib import Path
# Se define una matriz de numpy con valor de color inicial
import depthai as dai
import math


class Centroide():
    def __init__(self):
        self.caunt = 0
        self.x = 0
        self.y = 0
        self.color1_hsv = np.array([0, 0, 0])

        # ---- Load previous calibration file ----
        self.lower_init, self.upper_init, self.punto_de_referencia, \
        self.color1_hsv, self.ksize_init = self.cargar_valores_desde_archivo(
            "valores_lower_upper_refpoint.txt"
        )

        self.LowerColorError = np.array([-40, -45, -75])
        self.UpperColorError = np.array([40, 45, 75])

        # ---- Load image (Linux-safe path) ----
        package_dir = Path(__file__).resolve().parent
        self.image = cv2.imread(str(package_dir / "hsv_color_map.png"))

        # ---- GUI Setup ----
        cv2.namedWindow("image")
        cv2.resizeWindow("image", 540, 480)
        cv2.moveWindow("image", 700, 100)

        cv2.createTrackbar("HMin", "image", 0, 179, self.nothing)
        cv2.createTrackbar("SMin", "image", 0, 255, self.nothing)
        cv2.createTrackbar("VMin", "image", 0, 255, self.nothing)
        cv2.createTrackbar("HMax", "image", 0, 179, self.nothing)
        cv2.createTrackbar("SMax", "image", 0, 255, self.nothing)
        cv2.createTrackbar("VMax", "image", 0, 255, self.nothing)
        cv2.createTrackbar("ksize", "image", 0, 50, self.nothing)

        cv2.setTrackbarPos("HMin", "image", self.lower_init[0])
        cv2.setTrackbarPos("SMin", "image", self.lower_init[1])
        cv2.setTrackbarPos("VMin", "image", self.lower_init[2])
        cv2.setTrackbarPos("HMax", "image", self.upper_init[0])
        cv2.setTrackbarPos("SMax", "image", self.upper_init[1])
        cv2.setTrackbarPos("VMax", "image", self.upper_init[2])
        cv2.setTrackbarPos("ksize", "image", self.ksize_init)

        # ---- DepthAI Pipeline (Fixed) ----
        print("Inicializando pipeline DepthAI...")

        self.pipeline = dai.Pipeline()

        # Create a ColorCamera node and an XLinkOut to stream frames to the host.
        cam = self.pipeline.create(dai.node.ColorCamera)
        try:
            cam.setBoardSocket(dai.CameraBoardSocket.RGB)
        except Exception:
            # Some devices/versions may not require or support setBoardSocket
            pass
        cam.setPreviewSize(640, 400)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        xout = self.pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("video")
        cam.preview.link(xout.input)

        # Start device and create output queue for host
        self.device = dai.Device(self.pipeline)
        self.videoQueue = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)

        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 640, 480)
        cv2.moveWindow("frame", 30, 100)
        cv2.setMouseCallback("frame", self._mouseEvent)

        self.ejecutar()


# --- Función para cargar valores previos ---
    def cargar_valores_desde_archivo(self, filename):
            ruta_archivo = Path("/home/diego/TitoSync/ros2_ws/src/perception_pkg/perception_pkg") / filename


            with open(ruta_archivo, "r") as file:
                lines = file.readlines()
                lower = np.array(eval(lines[0].split(":")[1].strip()))
                upper = np.array(eval(lines[1].split(":")[1].strip()))
                reference_point = np.array(eval(lines[2].split(":")[1].strip()))
                color1_hsv = np.array(eval(lines[3].split(":")[1].strip()))
                ksize_init = np.array(eval(lines[4].split(":")[1].strip()))

            return lower, upper, reference_point,color1_hsv, int(ksize_init)

# --- Eventos del mouse ---
    def _mouseEvent(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            hsv_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            self.color1_hsv = hsv_frame[y, x]
            cv2.setTrackbarPos("HMin", "image", max(self.color1_hsv[0] + self.LowerColorError[0], 0))
            cv2.setTrackbarPos("SMin", "image", max(self.color1_hsv[1] + self.LowerColorError[1], 0))
            cv2.setTrackbarPos("VMin", "image", max(self.color1_hsv[2] + self.LowerColorError[2], 0))
            cv2.setTrackbarPos("HMax", "image", min(self.color1_hsv[0] + self.UpperColorError[0], 179))
            cv2.setTrackbarPos("SMax", "image", min(self.color1_hsv[1] + self.UpperColorError[1], 255))
            cv2.setTrackbarPos("VMax", "image", 255)
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.punto_de_referencia = [x, y]

    def nothing(self,x):
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
                # Obtener valores HSV actuales
                hMin = cv2.getTrackbarPos("HMin", "image")
                sMin = cv2.getTrackbarPos("SMin", "image")
                vMin = cv2.getTrackbarPos("VMin", "image")
                hMax = cv2.getTrackbarPos("HMax", "image")
                sMax = cv2.getTrackbarPos("SMax", "image")
                vMax = cv2.getTrackbarPos("VMax", "image")
                ksize = cv2.getTrackbarPos("ksize", "image")

                lower = np.array([hMin, sMin, vMin])
                upper = np.array([hMax, sMax, vMax])

                hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, lower, upper)
                result = cv2.bitwise_and(self.image, self.image, mask=mask)
                cv2.imshow("image", result)

                videoIn = self.videoQueue.get()
                if videoIn is None:
                    continue

                self.frame = videoIn.getCvFrame()

                self.frame = cv2.blur(self.frame, (ksize, ksize))
                frameHSV = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(frameHSV, lower, upper)
                contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                self.dibujar_punto_de_ref(self.frame)

                Contornos_Pos = []
                for c in contornos:
                    area = cv2.contourArea(c)
                    if area > 1000:
                        M = cv2.moments(c)
                        if M["m00"] == 0:
                            M["m00"] = 1
                        x = int(M["m10"] / M["m00"])
                        y = int(M["m01"] / M["m00"])
                        distancia = ((x - self.punto_de_referencia[0]) ** 2 + (y - self.punto_de_referencia[1]) ** 2) ** 0.5
                        cv2.circle(self.frame, (x, y), 7, (255, 0, 255), -1)
                        Contornos_Pos.append([c, distancia, (x, y)])

                Contornos_Pos.sort(key=lambda x: x[1])

                if Contornos_Pos:
                    _, _, centroide = Contornos_Pos[0]
                    self.x, self.y = centroide
                    dx = self.punto_de_referencia[0] - self.x 
                    dy = self.punto_de_referencia[1] - self.y
                    distancia = math.sqrt(dx**2 + dy**2)
                    angulo_rad = math.atan2(dy, dx)
                    angulo_deg = 90 - math.degrees(angulo_rad)
                    if angulo_deg > 180:
                        angulo_deg -= 360
                    elif angulo_deg <= -180:
                        angulo_deg += 360
                    cv2.line(self.frame, tuple(self.punto_de_referencia), (self.x, self.y), (0, 255, 255), 2)
                    cv2.putText(self.frame, f"Centroide: {self.x},{self.y},{round(distancia)},{round(angulo_deg)}deg)", (self.x + 10, self.y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                    
                    dx = self.punto_de_referencia[0] - self.x 
                    dy = self.punto_de_referencia[1] - self.y
                    distancia = math.sqrt(dx**2 + dy**2)
                    # Ángulo en radianes (puede ser negativo)
                    angulo_rad = math.atan2(dy, dx)

                    # Convertir a grados
                    
                    angulo_deg = 90 - math.degrees(angulo_rad)
                    self.caunt += 1
                    if self.caunt >= 100:   
                        self.caunt = 0            
                        print(f"Centroide detectado en: ({x}, {y})")
                        print(f"punto de referencia: ({self.punto_de_referencia[0]}, {self.punto_de_referencia[1]})")
                        print(f"dinstancia: ({distancia})")
                        print(f"angulo: ({angulo_deg})")

                cv2.imshow("frame", self.frame)

                key = cv2.waitKey(1)
                if key == ord("s"):
                    break

        except KeyboardInterrupt:
            pass

        finally:
            # --- Guardar valores actuales en archivo ---
            hMin = cv2.getTrackbarPos("HMin", "image")
            sMin = cv2.getTrackbarPos("SMin", "image")
            vMin = cv2.getTrackbarPos("VMin", "image")
            hMax = cv2.getTrackbarPos("HMax", "image")
            sMax = cv2.getTrackbarPos("SMax", "image")
            vMax = cv2.getTrackbarPos("VMax", "image")
            ksize = cv2.getTrackbarPos("ksize", "image")

            lower = [hMin, sMin, vMin]
            upper = [hMax, sMax, vMax]
            ruta_archivo = Path("/home/diego/TitoSync/ros2_ws/src/perception_pkg/perception_pkg") / "valores_lower_upper_refpoint.txt"
            with open(ruta_archivo, "w") as f:
                f.write(f"lower: {lower}\n")
                f.write(f"upper: {upper}\n")
                f.write(f"punto_de_referencia: {[int(self.punto_de_referencia[0]),int(self.punto_de_referencia[1])]}\n")
                f.write(f"color_a_rastrear: {self.color1_hsv.tolist()}\n")
                f.write(f"ksize: {ksize}\n")

            # --- Liberar recursos ---
            cv2.destroyAllWindows()

    
if __name__ == "__main__":
    detector = Centroide()
