#!/usr/bin/env python

from math import *
import cmath
from geometry_msgs.msg import Pose, Quaternion

class ComputeIk():

    def __init__(self):
        
        # Parametros iniciales
        self.BASE     = 110.0
        self.FOOT     =  40.0
        self.SHOULDER = 127.0
        self.KNEE     =  26.0
        self.ELBOW    = 133.0
        self.SPAN     = Math.hypot ELBOW, KNEE
        self.GRIPPER  = 121.0

        #self.a_2 = 
        #self.d_4 = 

    def compute_ik(self, p):
        
        # Initialization
        p_x = 5
        p_y = 0
        #p_z = 
        #a_2 = 
        #d_4 = 

        #TENGO QUE ESCRIBIRLO PARA PYTHON

        # theta1
	theta1=math.atan2(p_y,p_x)
        

        # theta3
        elbow_angle = 0.5 * Math::PI - cosinus_theorem(arm_vector.norm, SHOULDER, SPAN) + Math.atan(KNEE / ELBOW)

        # theta2
        shoulder_angle = arm_elevation + elbow_elevation - 0.5 * Math::PI

        # matriz de la cabeza del manipulador
        head_matrix = Matrix.rotate_y(shoulder_angle - elbow_angle) * Matrix.rotate_z(-base_angle) * matrix
        gripper_vector = head_matrix.z
	# theta4 & theta5
        pitch_angle = Math.atan2 Math.hypot(gripper_vector[1], gripper_vector[2]), gripper_vector[0]
	if Math.hypot(gripper_vector[1], gripper_vector[2]) < 1e-5
          roll_angle = 0
        elsif gripper_vector[2] >= 0
          roll_angle = Math.atan2 -gripper_vector[1], gripper_vector[2]
        else
          roll_angle = Math.atan2 gripper_vector[1], -gripper_vector[2]
          pitch_angle = -pitch_angle
        end


        # theta6
	adapter_matrix = Matrix[[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 0]]
        wrist_matrix = Matrix.rotate_x(-pitch_angle) * Matrix.rotate_z(-roll_angle) * adapter_matrix * head_matrix
        wrist_vector = wrist_matrix.x
        wrist_angle = Math.atan2 wrist_vector[1], wrist_vector[0]

        return [theta1, theta2, theta3-theta2, theta4, theta5, theta6]

if __name__ == '__main__':
    
    ik = ComputeIk()
    pose = Pose()
    pose.position.x = 
    pose.position.y = 
    pose.position.z = 
    thetas = ik.compute_ik(pose)
    for i in range(6):
        print thetas[i]
