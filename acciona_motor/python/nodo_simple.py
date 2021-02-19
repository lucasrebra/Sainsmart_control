#!/usr/bin/env python

import math
import rospy
import std_msgs.msg
import roslib
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

#Create our msg
posicion=Int32MultiArray()
for i in range(16):
    posicion.data.append(-1)
rospy.loginfo("Array rellenado, SEX")
radtodegree=180/math.pi
"""
#SERVO LIMITS 

#Limits PWM and angle for joint 1
zero_pwm1=280
min_pwm1=100
max_pwm1=500

zero_angle1=0
min_angle1=-90
max_angle1=90

#Limits PWM and angle for joint 2
zero_pwm2=280
min_pwm2=440
max_pwm2=150

zero_angle2=0
min_angle2=-81
max_angle2=72

#Limits PWM and angle for joint 3
#The limits of this joint depends also on the position ofjoint 2, see later
zero_pwm3=120
min_pwm3=100
max_pwm3=500

zero_angle3=0
min_angle3=-5
max_angle3=150

#Limits PWM and angle for joint 4
zero_pwm4=280
min_pwm4=100
max_pwm4=500

zero_angle4=0
min_angle4=-90
max_angle4=90"""

#Vector for PWMman , PWMmin, max angles and min angles
#The servo 5 now isn't working 
zero_pwm=[280,280,120,280,0,250]
min_pwm=[100,440,100,100,0,100]
max_pwm=[500,150,500,500,0,500]
min_angle=[-90,-81,-5,-90,0,-90]
max_angle=[90,72,150,90,0,90]

def callback(jointstate):
    global posicion
    #Calculamos
    posicion.data[0]=(jointstate.position[0]*radtodegree-min_angle[0])*(max_pwm[0]-min_pwm[0])/(max_angle[0]-min_angle[0])+min_pwm[0]
#jointstate.position[0]*radtodegree*(max_pwm1-min_pwm1)/(max_angle1-min_angle1)+min_pwm1
    posicion.data[1]=(jointstate.position[1]*radtodegree-min_angle[1])*(max_pwm[1]-min_pwm[1])/(max_angle[1]-min_angle[1])+min_pwm[1]
    #posicion.data[1]=jointstate.position[1]*(-1.80555555)+280
#jointstate.position[1]*radtodegree*(max_pwm2-min_pwm2)/(max_angle2-min_angle2)+min_pwm2
    posicion.data[2]=((jointstate.position[2]+jointstate.position[1])*radtodegree-min_angle[2])*(max_pwm[2]-min_pwm[2])/(max_angle[2]-min_angle[2])+min_pwm[2]
#jointstate.position[2]*radtodegree*(max_pwm3-min_pwm3)/(max_angle3-min_angle3)+min_pwm3
    posicion.data[3]=(jointstate.position[3]*radtodegree-min_angle[3])*(max_pwm[3]-min_pwm[3])/(max_angle[3]-min_angle[3])+min_pwm[3]
     
		posicion.data[4]=(jointstate.position[4]*radtodegree-min_angle[4])*(max_pwm[4]-min_pwm[4])/(max_angle[4]-min_angle[4])+min_pwm[4]

		posicion.data[5]=(jointstate.position[5]*radtodegree-min_angle[5])*(max_pwm[5]-min_pwm[5])/(max_angle[5]-min_angle[5])+min_pwm[5]
#If the angles are between limits OK, if not PWM max or me depending
#    for n in range(3):
 #       if posicion.data[n]> max_pwm[n]:
   #         posicion.data[n]=max_pwm[n]
      #  if posicion.data[n]< min_pwm[n]:
        #    posicion.data[n]=min_pwm[n]
   

def posicionador():
    pub=rospy.Publisher('/command',Int32MultiArray,queue_size=10)
    rospy.init_node('generaPWM',anonymous=False)
    freq=50
    rate=rospy.Rate(50)

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


