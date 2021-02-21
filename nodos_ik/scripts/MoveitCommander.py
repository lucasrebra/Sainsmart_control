#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


group_name = "Brazo"
group = moveit_commander.MoveGroupCommander(group_name)

# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print "============ Reference frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print "============ End effector: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print "============ Robot Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print "============ Printing robot state"
print robot.get_current_state()
print ""

joint_goal = group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0
joint_goal[3] = 0
joint_goal[4] = 0
joint_goal[5] = 0

plan=group.plan(joint_goal)
group.execute(plan,wait=True)
time.sleep(10)
group.stop()


#joint_goal = group.get_current_joint_values()
joint_goal[0] = pi/4
joint_goal[1] = pi/6
joint_goal[2] = 0
joint_goal[3] = -pi/8
joint_goal[4] = 0
joint_goal[5] = pi/4

plan=group.plan(joint_goal)
group.execute(plan,wait=True)
time.sleep(10)

group.stop()

joint_goal[0] = -pi/4
joint_goal[1] = pi/6
joint_goal[2] = 0
joint_goal[3] = -pi/8
joint_goal[4] = 0
joint_goal[5] = pi/4

plan=group.plan(joint_goal)
group.execute(plan,wait=True)

time.sleep(10)

group.stop()

joint_goal = group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0
joint_goal[3] = 0
joint_goal[4] = 0
joint_goal[5] = 0

plan=group.plan(joint_goal)
group.execute(plan,wait=True)
time.sleep(10)
group.stop()


