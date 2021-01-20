#!/usr/bin/env python

import rospy
import std_msgs.msg
import roslib
from sensor_msgs import JointState
from std_msgs import Int32MultiArray

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

    for(i=0;i<3;i=i+1):
    if(comando.data[i]< MINY)
        comando.data[i]=MINY
    if(comando.data[i]> MAXY)
        comando.data[i]=MAXY


def nodo():
    """Instantializate the node and run the loop"""

    comando=Int32MultiArray()

    pub=rospy.Publisher('command',Int32MultiArray,queue_size=10)

    rospy.init_node('CreaComandos',anonymous=False)

    rate=rospy.Rate(7.8125) #Rate(Hz)

    while not rospy.is_shutdown():
        """Create subscriber and run callback function each time"""

        rospy.Subscriber("myrobot/JointStates", JointState, callback)

        pub.publish(comando)

        rate.sleep()

if __name__=='__main__':
    """If there isn't exception run the node"""

    try:
        nodo()
    except rospy.ROSInterruptException:
        pass



