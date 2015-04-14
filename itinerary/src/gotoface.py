#!/usr/bin/env python

import roslib;
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
from std_msgs.msg import *

class GoFace():
	def __init__(self):
		rospy.init_node('goface',anonymous=True)

		rospy.on_shutdown(self.shutdown)
		
		self.rest_time = rospy.get_param("~rest_time",10)
		self.fake_test = rospy.get_param("~fake_test",False) 
		goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                               'SUCCEEDED', 'ABORTED', 'REJECTED',
                               'PREEMPTING', 'RECALLING', 'RECALLED','LOST']

		self.move_base = actionlib.SimpleActionClient("move_base",MoveBaseAction)

		rospy.loginfo("Waiting for move_base action server...")

		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
		self.move_base.wait_for_server(rospy.Duration(60))

		rospy.loginfo("Connected to move base server")

		# A variable to hold the initial pose of the robot to be set by 
		# the user in RViz
		self.initial_pose = PoseWithCovarianceStamped()

		rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
		rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
		self.last_location = Pose()
		rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
		self.goalpub = rospy.Publisher('thegoal',Pose)
		while self.initial_pose.header.stamp == "":
		    rospy.sleep(1)
		self.goal = MoveBaseGoal()
		rospy.loginfo("Starting navigation test")
		rospy.Subscriber("/destination_face",Pose,self.faceCallback)

	def faceCallback(self,data):
		rospy.loginfo("Going to see your face !");
		self.goal.target_pose.pose = data
		self.goal.target_pose.pose.orientation.x = 0
		self.goal.target_pose.pose.orientation.y = 0
		self.goal.target_pose.pose.orientation.w = 0.745572167589
		self.goal.target_pose.pose.orientation.z = -0.666424896681  
		rospy.loginfo("goal.x" + str(self.goal.target_pose.pose.position.x))
		rospy.loginfo("goal.y" + str(self.goal.target_pose.pose.position.y))
		rospy.loginfo("goal.z" + str(self.goal.target_pose.pose.position.z))

		self.goal.target_pose.header.frame_id = 'map'
		self.goal.target_pose.header.stamp = rospy.Time.now()
		self.move_base.send_goal(self.goal)
		self.goalpub.publish(self.goal.target_pose.pose);
		finished_within_time =  self.move_base.wait_for_result(rospy.Duration(100))
		if not finished_within_time:
			self.move_base.cancel_goal()
			rospy.loginfo("Can't achieve goal")
		else:
			rospy.loginfo("Alright ! Got you !");
	def shutdown(self):
		rospy.loginfo("Stopping the robot")
		self.move_base.cancel_goal()
		rospy.sleep(2)
		self.cmd_vel_pub.publish(Twist())
		rospy.sleep(1)
	def update_initial_pose(self,initial_pose):
		self.initial_pose = initial_pose

if __name__ == '__main__':
	try:
		GoFace()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("NOPE")	
