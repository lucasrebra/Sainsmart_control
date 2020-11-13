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

joint = JointState()
conv = math.pi / 180


def callback(posicion):
    """En la funcion callback inicializa los objetos ComputeIk y Pose en ik
    y en pose. luego calculamos los angulos de rotacion y los guardamos
    en thetas para luego publicar en joint con la estructura adecuada"""

    ik = ComputeIk()  # creamos un objeto de cinematica inversa

    pose = Pose()  # Creamos un objeto con estructura de formato Pose (POS: x y z w
# , ORIENTACION: xyz)
    pose.position.x = posicion.p_x
    pose.position.y = posicion.p_y
    pose.position.z = posicion.p_z

    # Calculamos IK y lo guardamos en thetas, valores que utilizaremos para
# publicar en rviz
    thetas = ik.calcular_ik(pose)

    # Datos para publicar con estructura de mensaje JointState
    joint.header = std_msgs.msg.Header()
    joint.header.stamp = rospy.Time.now()
    joint.name = ['joint1', 'joint2', 'joint3', 'joint4']
    joint.position = [thetas[1], thetas[2], thetas[3], thetas[4]]
    joint.velocity = []
    joint.effort = []


def nodo():
    
	pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
	rospy.init_node("publicaIk")
	rate = rospy.Rate(7.8125)

	while not rospy.is_shutdown():
		rospy.Subscriber("pos_robot", posicion, callback)
		pub.publish(joint)
		rate.sleep()


if __name__ == '__main__':
	try:
		nodo()
	except rospy.ROSInterruptException:
		pass
