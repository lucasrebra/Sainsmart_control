#!/usr/bin/env python


#Moveitcommander
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time
# importamos librerias que necesitaremos de ros
import roslib

# Funciones creadas para calculo de IK
from Funciones_FkIk import *

# Mensajes que necesitaremos para comunicacion entre nodos
from nodos_ik.msg import angulos
from sensor_msgs.msg import JointState
from std_msgs.msg import *
from nodos_ik.msg import posicion
from vision_trabajo.msg import Vectorpos

joint = JointState()
conv = math.pi / 180


def callback(Vectorpos):
    """En la funcion callback inicializa los objetos ComputeIk y Pose en ik
    y en pose. luego calculamos los angulos de rotacion y los guardamos
    en thetas para luego publicar en joint con la estructura adecuada"""
		

    ik = ComputeIk()  # creamos un objeto de cinematica inversa

    pose = Pose()  # Creamos un objeto con estructura de formato Pose (POS: x y z w
# , ORIENTACION: xyz)
    for i in range(Vectorpos.n):
			pose.position.x = Vectorpos.x[i]
			pose.position.y = Vectorpos.y[i]
			pose.position.z = Vectorpos.z[i]

     # Calculamos IK y lo guardamos en thetas, valores que utilizaremos para
     # publicar en rviz
			thetas = ik.calcular_ik(pose)


			#MoveitCommander
			
			joint_goal = group.get_current_joint_values()
			joint_goal[0] = thetas[0]
			joint_goal[1] = thetas[1]
			joint_goal[2] = thetas[2]
			joint_goal[3] = thetas[3]
			joint_goal[4] = thetas[4]
			joint_goal[5] = thetas[5]
			
			#Planeamos y ejecutamos similar RVIZ
			plan=group.plan(joint_goal)
			group.execute(plan,wait=True)
			time.sleep(10)
			group.stop()


		
		
		

def nodo():
	moveit_commander.roscpp_initialize(sys.argv)
	
	pub = rospy.Publisher('/hola', JointState, queue_size=10)
	rospy.init_node('publicaIk')

	#Iniciamos objetos de MoveitCommander
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group_name = "Brazo"
	group = moveit_commander.MoveGroupCommander(group_name)
	rate = rospy.Rate(0.1)

	joint_goal = group.get_current_joint_values()
	joint_goal[0] = 0.5
	joint_goal[1] = 0.1
	joint_goal[2] = 0
	joint_goal[3] = 0.3
	joint_goal[4] = 0
	joint_goal[5] = 0
	
	#Planeamos y ejecutamos similar RVIZ
	plan=group.plan(joint_goal)
	group.execute(plan,wait=True)
	time.sleep(10)
	group.stop()

	while not rospy.is_shutdown():
		rospy.Subscriber("/circulos/coordenadas", Vectorpos, callback)
		pub.publish(joint)
		rate.sleep()


if __name__ == '__main__':
	try:
		nodo()
	except rospy.ROSInterruptException:
		pass
