#!/usr/bin/env python

# importamos librerias que necesitaremos de ros
import rospy
import roslib

import math

# Funciones creadas para calculo de IK
from Funciones_FkIk import *

# Mensajes que necesitaremos para comunicacion entre nodos
from nodos_ik.msg import angulos
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from nodos_ik.msg import posicion


def ChivaPos():
    
	pub = rospy.Publisher('/pos_robot',posicion, queue_size=10)
	rospy.init_node("publicapos")
	rate = rospy.Rate(0.1)#7.8125
	vectorpos=[100,100,100]
	sol=posicion()
	sol.p_x=vectorpos[0]
	sol.p_y=vectorpos[1]
	sol.p_z=vectorpos[2]

	while not rospy.is_shutdown():
		rospy.loginfo(sol)
		pub.publish(sol)
		rate.sleep()


if __name__ == '__main__':
	try:
		ChivaPos()
	except rospy.ROSInterruptException:
		pass
