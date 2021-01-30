#!/usr/bin/env python

import rospy
import std_msgs.msg
import roslib

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

# MAXY --> maximo PWM
# MINY --> minimo PWM
# MAX_ANGLE --> maximo angulo alcanzable servo
# MIN_ANGLE --> minimo angulo alcanzable servo

def callback(MultiArray):
    """Function running in a rate of 50HZ"""

    #Calculamos
    comando.data[0]=MultiArray[0]*(MAXY-MINY)/(MAX_ANGLE-MIN_ANGLE)+MINY#Faltan unidades de conversion de radianes a angulos
    comando.data[1]=MultiArray[1]*(MAXY-MINY)/(MAX_ANGLE-MIN_ANGLE)+MINY
    comando.data[2]=MultiArray[2]*(MAXY-MINY)/(MAX_ANGLE-MIN_ANGLE)+MINY

		
		#Loop for knowing if it's between the limits and if not set to max or min

    for i in comando.data:

		  if(i< MINY):

		      comando.data[contador]=MINY

		  if(i> MAXY):

		      comando.data[contador]=MAXY


def nodo():
		pub=rospy.Publisher("/command",Int32MultiArray,queue_size=10)

		comando=Int32MultiArray()

		rospy.init_node('CreaComandos',anonymous=False)

		rospy.loginfo("CreaComandos node has initialized...")

		rate=rospy.Rate(0.1) #Rate(Hz)

		while not rospy.is_shutdown():

				rospy.Subscriber("/myrobot/joint_states", JointState, callback)

				pub.publish(comando)

				rate.sleep()

if __name__=='__main__':
    """If there isn't exception run the node"""

    try:
        nodo()
    except rospy.ROSInterruptException:
        pass



