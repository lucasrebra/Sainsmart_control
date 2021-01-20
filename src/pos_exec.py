#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
from std_msgs.msg import Int16
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_pos_interface', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
positions = [[0,0,0,0,0,0]]
for pos in positions:
	group_variable_values = group.get_current_joint_values()
	group_variable_values[0] = pos[0]
	group_variable_values[1] = pos[1]
	group_variable_values[2] = pos[2]
	group_variable_values[3] = pos[3]
	group_variable_values[4] = pos[4]
	group_variable_values[5] = pos[5]
	group.set_joint_value_target(group_variable_values)
	group.set_planning_time(10);
	plan = group.plan()
	group.go(wait=True)
	rospy.sleep(2)

moveit_commander.roscpp_shutdown()
