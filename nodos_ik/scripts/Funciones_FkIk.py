#!/usr/bin/env python

from math import *
import math
from geometry_msgs.msg import Pose, Quaternion
import numpy as np


def Teorema_coseno(a,b,c):
	"""Nos permite calcularlo sin falta de meter la formula cada vez"""
	angulo = math.acos((a*a+b*b-c*c)/(2*a*b))

	return angulo


def modVector(x, y, z):
	"""Modulo del vectr para calcular su longitus"""
	modulo=math.sqrt(x*x+y*y+z*z)
	return modulo


class ComputeIk():
	"""Esta clase sera la encargada de realizar el calculo IK"""
	def __init__(self):

		self.BASE=110.0
		self.FOOT=40.0
		self.SHOULDER=127.0
		self.KNEE=26.0
		self.ELBOW=133.0
		self.SPAN=math.hypot(self.ELBOW,self.KNEE)

	def calcular_ik(self,p):


		theta1=math.atan2(p.position.y,p.position.x)

		wrist_position=np.array([p.position.x,p.position.y,p.position.z])

		arm_vector=wrist_position-np.array([math.cos(theta1)*self.FOOT,math.sin(theta1)*self.FOOT,self.BASE])

		arm_elevation=math.atan2(arm_vector[2],math.hypot(arm_vector[0],arm_vector[1]))

		modulo_vector=modVector(arm_vector[0],arm_vector[1],arm_vector[2])

		elbow_elevation=Teorema_coseno(modulo_vector,self.SHOULDER,self.SPAN)

		shoulder_angle= -math.pi*0.5+elbow_elevation+arm_elevation

		angulo_aux=Teorema_coseno(modulo_vector,self.SHOULDER,self.SPAN)

	 	elbow_angle=pi/2-Teorema_coseno(self.SHOULDER,self.SPAN,modulo_vector)-math.atan2(KNEE,ELBOW)

		thetha4=0
		
		theta=[theta1, shoulder_angle, elbow_angle,theta4]
	
	
		return theta


"""

COMPROBADO EN PYTHON TABIÉN
Px=150*10^-3
Py=150*10^-3
Pz=50*10^-3

FOOT=40*10^-3
BASE=110*10^-3
brazo=127*10^-3
hombro=133*10^-3
KNEE=26*10^-3
SPAN=sqrt(hombro^2+KNEE^2)

tetha1=atan2(Py,Px)

Px=(150*10^-3)-40*(10^-3)*cos(tetha1)
Py=(150*10^-3)-40*(10^-3)*sin(tetha1)
Pz=(50*10^-3)-BASE

modulo=sqrt((Px)^2+(Py)^2+(Pz)^2)
coseno=(-0.127^2-SPAN^2+modulo^2)/(2*SPAN*0.127)%teorema coseno
%mejor con el SPAN en vez de con 133, si no falla
%posible error
if (coseno>1)
    print("NO HAY SOLUCION")
else
tetha3=pi/2-acos((0.127^2+SPAN^2-modulo^2)/(2*SPAN*0.127))-atan2(KNEE,hombro)

elevacion=atan2(Pz,sqrt(Px^2+Py^2))
hola=(modulo^2+brazo^2-SPAN^2)/(2*modulo*brazo)
gamma=acos(hola)

tetha2=-pi/2+elevacion+gamma

tethas=[tetha1 tetha2 tetha3]
end

"""
	

