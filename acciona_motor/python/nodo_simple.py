#!/usr/bin/env python

import math
import rospy
import std_msgs.msg
import roslib
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

#Create our msg
posicion=Int32MultiArray()
for i in range(16):
    posicion.data.append(-1)
rospy.loginfo("Array rellenado, SEX")
radtodegree=180/math.pi

#Servo limits
min_pwm=172
max_pwm=565
max_angle=180
min_angle=0

def callback(jointstate):
    global posicion
    #Calculamos
    posicion.data[0]=jointstate.position[0]*180/math.pi*(max_pwm-min_pwm)/(max_angle-min_angle)+min_pwm
    posicion.data[1]=jointstate.position[1]*180/math.pi*(max_pwm-min_pwm)/(max_angle-min_angle)+min_pwm
    posicion.data[2]=jointstate.position[2]*180/math.pi*(max_pwm-min_pwm)/(max_angle-min_angle)+min_pwm

    #Loop for knowing if it's between the limits and if not set to max or min

    for i in range(16):
        if posicion.data[i] < min_angle:

            posicion.data[i] = min_angle

        if posicion.data[i] > max_angle:

            posicion.data[i] = max_angle

        if posicion.data[i] == -1:

            posicion.data[i] = (self.min_angle+self.max_angle)/2    
        

def posicionador():
    pub=rospy.Publisher('/command',Int32MultiArray,queue_size=10)
    rospy.init_node('generaPWM',anonymous=False)
    freq=50
    rate=rospy.Rate(freq)

    while not rospy.is_shutdown():
        rospy.Subscriber("/myrobot/joint_states",JointState,callback)
        pub.publish(posicion)
        rate.sleep()

if __name__=='__main__':
    """If there isn't exception run the node"""

    try:
        posicionador()
    except rospy.ROSInterruptException:
        pass


