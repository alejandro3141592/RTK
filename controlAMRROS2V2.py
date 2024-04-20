#!/usr/bin/env python
import cv2
import numpy as np
import math
import pandas as pd
from geopy.distance import geodesic
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int16


dataFrame = pd.read_csv("/home/alejandro/ros2_ws/src/RTK/RTK-manejo-autnomo/Documents/CSV/basep.csv")
print("\nReading the CSV file...\n", dataFrame)

long = dataFrame['Longitud'].values.tolist()
lat = dataFrame['Latitud'].values.tolist()
print("Longitude: ", long)
print("min: ", min(long))
print("max: ", max(long))
print("Latitude: ", lat)
print("min: ", min(lat))
print("max: ", max(lat))
print(len(long))
print(len(lat))



cartesianasImagenX = []
cartesianasImagenY = []

cartesianasX = []
cartesianasY = []

image_path = r'/home/alejandro/ros2_ws/src/RTK/RTK-manejo-autnomo/Documents/Photo/Foto1.png'

image = cv2.imread(image_path)
height = image.shape[0]
width = image.shape[1]
print("height: ", height)
print("width: ", width)

def coordenadasApixeles(latitudes, longitudes):
    for i in range(len(long)):
        Ximagen, Yimagen = polar_coordinates_2_pixels(latitudes[i], longitudes[i])
        cartesianasImagenX.append(Ximagen)
        cartesianasImagenY.append(Yimagen)

def coordenadasACartesianas(latitudes, longitudes):
    for i in range(len(long)):
        Xcartesiana, Ycartesiana = polar_coordinates_2_cartesians(latitudes[i], longitudes[i])
        cartesianasX.append(Xcartesiana)
        cartesianasY.append(Ycartesiana)


def polar_coordinates_2_pixels(latitude, longitude):
    x_cartesian, y_cartesian = polar_coordinates_2_cartesians(latitude, longitude)

    x = int((1778.9356 * x_cartesian) + 1536723.11)
    y = int((-1878.46*y_cartesian) + 3900407.83)
    return x, y

def polar_coordinates_2_cartesians(latitude, longitude):
    R = 6371
    x = R * math.cos(math.radians(latitude)) * math.cos(math.radians(longitude))
    y = R * math.sin(math.radians(latitude))
    return x, y
    

coordenadasApixeles(lat, long)
coordenadasACartesianas(lat, long)
print("X pixel ", cartesianasImagenX)
print("Y pixel ", cartesianasImagenY)
print("X cartesiana ", cartesianasX)
print("Y cartesiana ", cartesianasY)

n = 0
p = 0 + n


def anguloVectores(inicioX, inicioY, Ax, Ay, Bx, By):
    punto = np.dot([(Ax-inicioX), (inicioY-Ay)], [(Bx-inicioX), (inicioY-By)])
    distanciaA = math.sqrt(((Ax-inicioX)**2) + ((inicioY-Ay)**2))
    distanciaB = math.sqrt(((Bx-inicioX)**2) + ((inicioY-By)**2))
    distancias = distanciaA * distanciaB
    anguloRadianes = math.acos(min(max(punto / distancias, -1), 1))
    anguloV = math.degrees(anguloRadianes)
    return anguloV

def anguloDesfase(inicioX, inicioY, Ax, Ay, Bx, By):
    actual = math.degrees(math.atan2((Ay-inicioY),(Ax-inicioX)))
    objetivo = math.degrees(math.atan2((By-inicioY),(Bx-inicioX)))
    ObjetivoActual = [(Bx-inicioX), (By-inicioY)]

    if actual < 0:
        actual += 360
    if objetivo < 0:
        objetivo += 360

    theta = math.radians(-(actual-90))
    MatrixRotacion = [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]]
    ObjetivoNuevo = np.matmul(MatrixRotacion, ObjetivoActual)
    prueba = math.degrees(math.atan2(ObjetivoNuevo[0], ObjetivoNuevo[1]))
    print('Prueba: ', end='')
    print(prueba)
    anguloAMR = int(np.clip((prueba + 50), 0, 100))
    return prueba, anguloAMR

def dibujarCoordenadas(pixelesX, pixelesY, imagenDibujar):
    for i in range(len(pixelesX)):                               # B   G    R
        cv2.circle(imagenDibujar, (pixelesX[i], pixelesY[i]), 2, (255, 0, 255), -1)


class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")


        self.latitud = 19.0183016
        self.longitud = -98.2402208
        self.magnetometro = 0

        self.pub_error = self.create_publisher(Float64, "errorAMR", 10)
        self.pub_direction = self.create_publisher(Int16, "direccionAMR", 10)
        self.pub_velocity = self.create_publisher(Int16, "velocidadAMR", 10)

        self.timer_ = self.create_timer(1, self.publish_commands)

        self.create_subscription(Float64, 'latitude', self.latitude_callback, 10)
        self.create_subscription(Float64, 'longitude', self.longitude_callback, 10)
        self.create_subscription(Float64, 'angulo_magnetometro', self.magnetometro_callback, 10)

        self.ErrorAnguloReal = Float64()

        self.AnguloControlAMR = Int16()

        self.velocidaAMR = Int16()
            

    def publish_commands(self):
        self.pub_error.publish(self.ErrorAnguloReal)
        self.pub_direction.publish(self.AnguloControlAMR )
        self.pub_velocity.publish(self.velocidaAMR )

    #Unir lat y long en un solo dato
    def latitude_callback(self, msg:Float64):
        global p, cartesianasX, cartesianasY, cartesianasImagenX, cartesianasImagenY
        self.latitud = msg.data
        pixel_x, pixel_y = polar_coordinates_2_pixels(self.latitud, self.longitud)
        cartesian_x, cartesian_y = polar_coordinates_2_cartesians(self.latitud, self.longitud)

        largo = 32
        x_vector_pixeles, y_vector_pixeles = self.vector_orientacion_pixeles(pixel_x, pixel_y, largo, self.magnetometro)  
        x_vector, y_vector =  self.vector_orientacion(cartesian_x, cartesian_y, 0.025, self.magnetometro)

        #anguloPuntoCaretesiano = anguloVectores(cartesian_x, cartesian_y, x_vector, y_vector, cartesianasX[p], cartesianasY[p])

        self.ErrorAnguloReal.data, self.AnguloControlAMR.data= anguloDesfase(cartesian_x, cartesian_y, x_vector, y_vector, cartesianasX[p], cartesianasY[p])
        print("Angulo Error: ", self.ErrorAnguloReal.data)
        print("Angulo AMR: ", self.AnguloControlAMR.data)


        coodenadas_AMR = (self.latitud, self.longitud)
        coordenadas_Waypoint = (lat[p], long[p])
        distanciaGeopy = geodesic(coodenadas_AMR, coordenadas_Waypoint).meters
        self.velocidaAMR.data = int(np.clip((distanciaGeopy / 0.035), 0, 255))
        print("Distancia Geopy: ", distanciaGeopy)
        print("Velocidad: ", self.velocidaAMR.data)

        self.publish_commands()

        #Para visualizar
        img = image.copy()

        dibujarCoordenadas(cartesianasImagenX, cartesianasImagenY, img)
        print('XP')
        print(y_vector_pixeles)
        cv2.line(img, (int(pixel_x), int(pixel_y)), (int(x_vector_pixeles), int(y_vector_pixeles)), (0, 255, 0), 1)
        cv2.line(img, (int(pixel_x), int(pixel_y)), (int(cartesianasImagenX[p]), int(cartesianasImagenY[p])), (0, 0, 255), 2)
        cv2.circle(img, (pixel_x, pixel_y), 2, (0, 165, 255), -1)
        cv2.rectangle(img, (0, 0), (200, 170), (255, 255, 255), -1)


        textoError = "Error: " + str(round(self.ErrorAnguloReal.data, 3))
        textoAMR = "AMR: " + str(self.AnguloControlAMR)
        textoDistancia = "Dist: " + str(round(distanciaGeopy, 3))
        textoVelocidad = "vAMR: " + str(self.velocidaAMR)

        cv2.putText(img, textoError, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(img, textoAMR, (5, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(img, textoDistancia, (5, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(img, textoVelocidad, (5, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 1, cv2.LINE_AA)

        cv2.imshow('Cartesian Map', img)

        ROI = img[(pixel_y-50):(pixel_y+50), (pixel_x-100):(pixel_x+100)]
        ROI = cv2.resize(ROI, (600, 300), interpolation=cv2.INTER_AREA)
        cv2.imshow('ROI', ROI)
        print(cv2.getWindowProperty('Cartesian Map', cv2.WND_PROP_VISIBLE))
        if distanciaGeopy < 1:
            p += 1
        if cv2.waitKey(1) and 0xFF == ord('q'):
            rclpy.shutdown()
            cv2.destroyAllWindows()


        self.get_logger().info(str(msg.data))

    def longitude_callback(self, msg:Float64):
        
        self.longitud = msg.data
        self.get_logger().info(str(msg.data))

    def magnetometro_callback(self, msg:Float64):
        
        self.magnetometro = msg.data
        self.get_logger().info(str(msg.data))


    

    def vector_orientacion_pixeles(self, inicio_x, inicio_y, largo, angulo):
        orientacionX = int((largo * math.cos(math.radians(angulo))) + inicio_x)
        orientacionY = int(inicio_y - (largo * math.sin(math.radians(angulo))))
        return orientacionX, orientacionY

    def vector_orientacion(self, inicio_x, inicio_y, largo, angulo):
        orientacionX = (largo * math.cos(math.radians(angulo))) + inicio_x
        orientacionY = inicio_y + (largo * math.sin(math.radians(angulo)))
        return orientacionX, orientacionY













def main(args = None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()