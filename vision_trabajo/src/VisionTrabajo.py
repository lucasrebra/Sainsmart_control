#!/usr/bin/env python
import cv2
import numpy as np
import math



def detectorCirculos(image):
	#Leemos la imagen y fijamos el centro de nuestro sistema
	#de coordenadas en el robot
	x_distances=[]
	y_distances=[]
	z_distances=[]
	n=0

	cv2.circle(image,(670,520),10,(0,0,0),-1)
	cv2.circle(image,(500,250),10,(0,150,0),-1)
	name = 'HOLA'

	#Propiedades que tendra nuestra imagen
	print("Image Properties")
	print("- Number of Pixels: " + str(image.size))
	print("- Shape/Dimensions: " + str(image.shape))


	#Fijamos los parametros entre los que se encuentran
	#nuestros colores previamente calibrados
	hsv_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	low_red = np.array([150, 70, 150])
	high_red = np.array([200, 300, 255])

	red_mask=cv2.inRange(hsv_frame,low_red,high_red)
	red = cv2.bitwise_and(image, image, mask=red_mask)

	imgray = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)


	#kernal=np.ones((5,5), "uint8")
	#cv2.filter2D(imgray,-1,kernal)

	'''Limpiamos lo que no sean nuestros objetos'''
	kernel = np.ones((5,5),np.uint8)
	kernelerosion= np.ones((10,10),np.uint8)
	dilation = cv2.dilate(imgray,kernel,iterations = 1)
	#erosion = cv2.erode(dilation,kernel,iterations=1)
	##erosion = cv2.erode(dilation,kernelerosion,iterations=2)
	#dilation = cv2.dilate(imgray,kernelerosion,iterations = 1)

	im2, contours,hierarchy = cv2.findContours(dilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	#low_green = np.array([25, 52, 72])
	#high_green = np.array([102, 255, 255])
	#green_mask = cv2.inRange(hsv_frame, low_green, high_green)
	#green = cv2.bitwise_and(frame, frame, mask=green_mask)

	image=cv2.drawContours(image, contours, -1, (0,255,0), 3)



	for contour in contours:
		area=cv2.contourArea(contour)
		if area>1200:
			x,y,w,h = cv2.boundingRect(contour)
			#Vamos a recorrer la imagen realizando las transformaciones de 
			#los ejes x e y transformados segun la calibracion empleada
			print("pixels en x ----->" + str((2*x+w)/2-675))
			print("pixels en y ----->" + str((2*y+h)/2-500))
			print("width en x ------>" + str(w))
			print("heigh en y ------>" + str(h))

			x_d= -0.0852*((2*y+w)/2)+47.204 # 0.138
			y_d= -0.0876*((2*x+h)/2)+58.127  
			
			if math.sqrt(x_d**2+y_d**2) < 24:
				cv2.rectangle(image, (x,y), (x+w,y+h),(255,0,0),2)
				cv2.line(image,(670,520),((2*x+w)/2,(2*y+h)/2),(250,0,255),2)
				s = 'x_d:'+ str(x_d)+ 'y_d:'+str(y_d)
				cv2.putText(image,s,(x-20,y-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1)
				x_distances.append(x_d)
				y_distances.append(y_d)
				z_distances.append(0)
				n=n+1
				

			elif math.sqrt(x_d**2+y_d**2) > 24:
				cv2.rectangle(image, (x,y), (x+w,y+h),(0,0,255),2)
				s_error="OUT OF BOUNDARIES"
				cv2.putText(image,s_error,(x-20,y-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1)
			
			



	#Vamos a ir viendo las imagenes que van saliendo como salida
	#cv2.imshow('',image)
	#cv2.imshow("Red", dilation)
	#cv2.imshow("Mask",imgray)
	#cv2.imshow("green", green)

	#cv2.waitKey(0)

	return x_distances, y_distances, z_distances,n, image , red_mask

#test the function
#if __name__ == "__main__":

#	"""Funcion principal para probar funcion"""
#	image= "distancias1.png"
#	detectorCirculos(image)


