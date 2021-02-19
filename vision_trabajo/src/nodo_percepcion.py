#!/usr/bin/env python

import sys
import rospy
import cv2
import time

from std_msgs.msg             import String, Int32MultiArray
from sensor_msgs.msg          import Image
from cv_bridge                import CvBridge, CvBridgeError
from vision_trabajo           import *
from vision_trabajo.msg       import Vectorpos

class DetectaCirculos:

    def __init__(self):

        self.t0 = time.time()
        self.pos_circulos = Vectorpos()


        #Publicando en dos topicos en forma de imagen:
        #--> /circulos/imagen_deteccion
        #--> /circulos/objetos_detectados
        self.image_detect = rospy.Publisher("/circulos/imagen_deteccion",Image,queue_size=1)
        self.mask_pub = rospy.Publisher("/circulos/image_mask",Image,queue_size=1)

        #Publicamos tambien la posicion de nuestros circulos en el plano
        self.circle_position = rospy.Publisher("/circulos/coordenadas",Vectorpos,queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.callback)
        #self.image_sub = 'distancias1.png'

    def callback(self,data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
        
        x_circulo, y_circulo, image_drawn, mask = detectorCirculos(data)

        #Publicamos en los dos topicos de imagenes
        self.image_detect.publish(self.bridge.cv2_to_imgmsg(image_dawn,"bgr8"))
        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask,"8UC1"))

        #Guardamos los valores en con el formato de nuestro mensaje
        for i in range(3):
            self.pos_circulos.x[i] = 0# x_circulo[i]
            self.pos_circulos.y[i] = 0# y_circulo[i]
            self.pos_circulos.z[i] = 0 #En nuestro ejemplo esta en el suelo
        
        #publicamos la posicion de los circulos en /circulos/coordenadas
        self.circle_position.publish(self.pos_circulos)
            
        
        
if __name__ == '__main__':
    ic= DetectaCirculos()
    rospy.init_node('detector_circulos',anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
