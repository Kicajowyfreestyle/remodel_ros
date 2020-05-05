#!/usr/bin/env python2

from math import pi, sqrt

import rospy
from geometry_msgs.msg import Point, Quaternion

from tf.transformations import quaternion_from_euler
from arm_planner import ArmPlanner

rospy.init_node('demo_move')
rospy.sleep(2.)
left_arm = ArmPlanner("left")
right_arm = ArmPlanner("right")
i = 0

#position = Point(0.4, 0.2, 0.6)
#orientation = Quaternion(0., 0., 0., 1.)
#left_arm.move_pose(position, orientation)
#position = Point(0.4, -0.2, 0.6)
#a = list(quaternion_from_euler(0., 0., pi/2))
#orientation = Quaternion(*a)
#right_arm.move_pose(position, orientation)

left_arm.add_box("xD", Point(0., 0., 0.3), Quaternion(0., 0., 0., 1.), (0.2, 2.0, 0.6))
right_arm.add_box("xD", Point(0., 0., 0.3), Quaternion(0., 0., 0., 1.), (0.2, 2.0, 0.6))

z = 1.0
y = 0.3
positions_l = [Point(0.4, y, z + 0.1), Point(0.4, 0.2, z), Point(0.3, y + 0.1, z)]
orientations_l = [Quaternion(0., 0., 0., 1.), Quaternion(0., 0., 0., 1.), Quaternion(0., 0., -sqrt(2)/2, sqrt(2)/2)]
#orientations_l = [Quaternion(0., 0., 0., 1.), Quaternion(0., 0., 0., 1.), Quaternion(0., 0., 0., 1.)]
left_arm.move_path(positions_l, orientations_l)
positions_r = [Point(0.4, -y, z + 0.1), Point(0.4, -y, z), Point(0.3, -y - 0.1, z)]
#orientations_r = [Quaternion(0., 0., 0., 1.), Quaternion(0., 0., sqrt(2)/2, sqrt(2)/2), Quaternion(0., 0., 0., 1.)]
#orientations_r = [Quaternion(0., 0., 0., 1.), Quaternion(0., 0., 0., 1.), Quaternion(0., 0., 0., 1.)]
orientations_r = [Quaternion(0., 0., 0., 1.), Quaternion(0., 0., 0., 1.), Quaternion(0., 0., sqrt(2)/2, sqrt(2)/2)]
right_arm.move_path(positions_r, orientations_r)
