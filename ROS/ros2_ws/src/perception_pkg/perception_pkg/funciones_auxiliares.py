
from math import acos, degrees
from numpy import dot, linalg, ones, uint8
from cv2 import line, circle, bitwise_and

def mascara_circulo(frame, height, width, center_x, center_y):

    # Crear una máscara negra con las mismas dimensiones que el frame
    mask = ones((height, width), dtype=uint8) * 255

    # Coordenadas del centro y radio del círculo a enmascarar
    center = (center_x, center_y)  # Cambia estas coordenadas según sea necesario
    radius = 50  # Cambia el radio según sea necesario

    # Dibuja un círculo negro en la máscara
    circle(mask, center, radius, 0, -1)

    # Aplica la máscara al frame
    frame_nuevo = bitwise_and(frame, frame, mask=mask)

    return frame_nuevo


def grados_entre_vectores(vector_1, vector_2):
    # Calcular el producto escalar de los dos vectores
    dot_product = dot(vector_1, vector_2)
    # Calcular las magnitudes de los vectores
    magnitude1 = linalg.norm(vector_1)
    magnitude2 = linalg.norm(vector_2)
    # Calcular el coseno del ángulo usando la fórmula del producto escalar
    if magnitude1 == 0 or magnitude2 == 0:
        cos_theta = 0
    else:    
        cos_theta = dot_product / (magnitude1 * magnitude2)
    # Asegurarse de que el valor esté en el rango válido para acos
    cos_theta = max(-1.0, min(1.0, cos_theta))
    # Calcular el ángulo en radianes
    angulo_radianes = acos(cos_theta)
    # Convertir a grados si es necesario
    angulo_grados = degrees(angulo_radianes)
    return angulo_grados


def linea_discontinua(
    image,
    start_point,
    end_point,
    line_color,
    line_thickness=2,
    dash_length=10,
    gap_length=10,
):
    # Calcular la longitud total de la línea
    line_length = int(
        ((end_point[0] - start_point[0]) ** 2 + (end_point[1] - start_point[1]) ** 2)
        ** 0.5
    )

    # Calcular la cantidad total de segmentos
    num_segments = line_length // (dash_length + gap_length)

    # Dibujar cada segmento de la línea discontinua
    for i in range(num_segments):
        start_segment = (
            int(
                start_point[0]
                + (end_point[0] - start_point[0])
                * (i * (dash_length + gap_length))
                / line_length
            ),
            int(
                start_point[1]
                + (end_point[1] - start_point[1])
                * (i * (dash_length + gap_length))
                / line_length
            ),
        )

        end_segment = (
            int(
                start_point[0]
                + (end_point[0] - start_point[0])
                * ((i * (dash_length + gap_length)) + dash_length)
                / line_length
            ),
            int(
                start_point[1]
                + (end_point[1] - start_point[1])
                * ((i * (dash_length + gap_length)) + dash_length)
                / line_length
            ),
        )

        # Dibujar el segmento
        line(image, start_segment, end_segment, line_color, line_thickness)
