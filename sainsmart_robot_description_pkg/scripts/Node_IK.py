#!/usr/bin/env python
import delta_kinematics
import codos
import roslib
import rospy
import std_msgs.msg
from sensor_msgs.msg import JointState
from tfm_simulacion.msg import 

pi = 3.141592653
dtr = pi / 180.
joint = JointState ()

def callback ( data ):
	global joint
	# rospy.loginfo (' callback ')
	p=delta_kinematics.forward (data.theta1,data.theta2,data.theta3)
	punto=[p[1],p[2],p [3]]

        #Ejemplo de lo que se puede realizar con la FK
	c1=codos.punto_codo(data.theta1)
	p1=codos.punto_ee(punto,1)
	[a1_a,a1_b]=codos.angulos_codo(c1,p1,1)
	c2=codos.punto_codo(data.theta2)
	c2=codos. rotacion120(c2)
	p2 = codos . punto_ee (punto , 2)
	[a2_a , a2_b ] = codos . angulos_codo (c2 , p2 , 2)
	c3 = codos . punto_codo ( data . theta3 )
	c3 = codos . rotacion120 (c3)
	c3 = codos . rotacion120 (c3)
	p3 = codos . punto_ee (punto , 3)
	[a3_a , a3_b ] = codos . angulos_codo (c3 , p3 , 3)

	# Datos para publicar--> Se rellenan los datos para publicar, 
	joint . header = std_msgs .msg. Header ()
	joint . header . stamp = rospy . Time .now ()
	joint . name = ['base_brazo1', 'base_brazo2', 'base_brazo3', 				'codo1_a','codo1_b', 'codo2_a', 'codo2_b', 				'codo3_a', 'codo3_b', 'act_x', 'act_y', 'act_z']
	joint . position = [ data . theta1 * dtr , data . theta2 * dtr , data . theta3 *
	dtr , data . theta1 * dtr + a1_a , -a1_b , data . theta2 * dtr + a2_a , -a2_b ,
	data . theta3 * dtr + a3_a , -a3_b , punto [0] / 1000 , -punto [1] / 1000 ,
	punto [2] / 1000]
	joint . velocity = []
	joint . effort = []
