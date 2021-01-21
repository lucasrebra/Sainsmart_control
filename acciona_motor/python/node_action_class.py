#!/usr/bin/env python
import math
import rospy
import std_msgs.msg
import roslib
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

# MAXY --> maximo PWM
# MINY --> minimo PWM
# MAX_ANGLE --> maximo angulo alcanzable servo
# MIN_ANGLE --> minimo angulo alcanzable servo

class node_action_class(object):
    """Class of the object that will create our PWM by servo parameters"""

    def __init__(self):
        """Initialize the publisher and the parameters of the servos"""

        #Create our publisher in int32 multiarray format (/command)
        self.pub=rospy.Publisher("/command",Int32MultiArray,queue_size=10)
        #Create our msg
        self.pub_command=JointState()
        #fill in the message with -1 at first (no motion in our driver)
        for i in range(16):
            self.pub_command.position.append(-1) #full the vector 1 by 1
        
        rospy.loginfo("IntMultiarray initialized and full by -1")

        #time parameters
        self.freq=50 #Hz
        self.rate=rospy.Rate(self.freq)

        #Servo limits
        self.min_pwm=172
        self.max_pwm=565
        self.max_angle=180
        self.min_angle=0
    
    def run(self):
        """Program that will be run in the rate configured"""

        while not rospy.is_shutdown():
				
            #Calculamos
            self.pub.data[0]=self.pub_command.position[0]*180/math.pi*(self.max_pwm-self.min_pwm)/(self.max_angle-self.min_angle)+self.min_pwm
            self.pub.data[1]=self.pub_command.position[1]*180/math.pi*(self.max_pwm-self.min_pwm)/(self.max_angle-self.min_angle)+self.min_pwm
            self.pub.data[2]=self.pub_command.position[2]*180/math.pi*(self.max_pwm-self.min_pwm)/(self.max_angle-self.min_angle)+self.min_pwm

            #Loop for knowing if it's between the limits and if not set to max or min

            for i in range(16):
                if self.pub_command.data[i] < self.min_angle:

                    self.pub_command.data[i] = self.min_angle

                if self.pub_command.data[i] > self.max_angle:

                    self.pub_command.data[i] = self.max_angle

                if self.pub_command.data[i] == -1:

                    self.pub_command.data[i] = (self.min_angle+self.max_angle)/2
        


if __name__=='__main__':

    rospy.init_node('publish_pwm')
    rospy.loginfo("pwm_test_python_pub_node started ...")
    #Instantializate object
    pwm_object=node_action_class()
    pwm_object.sub=rospy.Subscriber("/myrobot/joint_state",JointState,pwm_object.run())
    pwm_object.pub.publish(pwm_object.pub_command)

    rospy.spin()
        




        




