#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

right_arm = rospy.Publisher('right/arm_controller/command', JointTrajectory, queue_size=10)
left_arm = rospy.Publisher('left/arm_controller/command', JointTrajectory, queue_size=10)
rospy.init_node('demo_move')
r = rospy.Rate(10) # 10hz
i = 0
while not rospy.is_shutdown():
   def make_move(pub, pt, ns):
      jt = JointTrajectory()
      jt.header.seq = i
      jt.header.stamp = rospy.Time.now()
      jt.header.frame_id = ''
      jtp = JointTrajectoryPoint()
      jtp.positions = pt
      #jtp.velocities = [0.]
      #jtp.accelerations = [0.]
      #jtp.effort = [0.]
      jtp.time_from_start = rospy.Duration(secs=2, nsecs=0)
      a = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
      a = [ns + x for x in a]
      jt.joint_names = a
      jt.points = [jtp]
      pub.publish(jt)

   make_move(left_arm, [1., 0., -0.7, 0., 0.5, 0.], "left_arm_")
   #make_move(left_arm, [0., 0., 0.0, 0., 0.0, 0.], "left_arm_")
   make_move(right_arm, [-1., -0.7, 0., 1., 0., 0.], "right_arm_")
   #make_move(right_arm, [0., 0., 0., 0., 0., 0.], "right_arm_")
   r.sleep()
   i += 1

