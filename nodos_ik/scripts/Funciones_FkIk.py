#!/usr/bin/env python

from math import *
import math
from geometry_msgs.msg import Pose, Quaternion
import numpy as np


def Teorema_coseno(a, b, c):
	"""Nos permite calcularlo sin falta de meter la formula cada vez"""
	angulo = math.acos((a * a + b * b - c * c) / 2 * a * b)

	return angulo


def modVector(x, y, z):
	"""Modulo del vectr para calcular su longitus"""
	modulo = math.sqrt(x * x + y * y + z * z)
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
		self.GRIPPER=121.0

	def calcular_ik(self,p):

		# Inicializacion de variables
		p_x = 50
		p_y = 0
		p_z = 30

		theta1=math.atan2(p.position.y,p.position.x)

		wrist_position=np.array([p.position.x,p.position.y,p.position.z+20])

		arm_vector=wrist_position-np.array([math.cos(theta1)*self.FOOT,math.sin(theta1)*self.FOOT,self.BASE])

		arm_elevation=math.atan2(arm_vector[2],math.hypot(arm_vector[0],arm_vector[1]))

		modulo_vector=modVector(arm_vector[0],arm_vector[1],arm_vector[2])

		elbow_elevation=Teorema_coseno(self.SPAN,modulo_vector,self.SHOULDER)

		shoulder_angle= math.pi*0.5-elbow_elevation-arm_elevation

		angulo_aux=Teorema_coseno(modulo_vector,self.SHOULDER,self.SPAN)

		elbow_angle=math.atan2((math.sin(arm_elevation+angulo_aux)*self.SHOULDER-p.position.z)/(sqrt(p.position.x*p.position.x+p.position.y*p.position.y)-math.cos(arm_elevation+angulo_aux)*self.SHOULDER))

		theta4=0
		
		theta=[theta1, shoulder_angle, elbow_angle,theta4]
		
		return theta


	

