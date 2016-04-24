#!/usr/bin/env python 
import roslib
roslib.load_manifest('learning_tf')
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import std_msgs.msg

class ApproachFace():
	def __init__(self):

		self.goal_sent = False

		#What to do if shut down (e.g. Ctrl-C or failure)
		rospy.on_shutdown(self.shutdown)

		# Tell the action client that we want to spin a thread by default
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Wait for the action server to come up")

		#Allow up to 5 seconds for the action server to come up
		self.move_base.wait_for_server(rospy.Duration(5))


		rospy.init_node('approach')
		rospy.Subscriber('map', String, test_cb)

	def test_cb(data):
		print(data)

	def goto(self, pos, quat):

		#Send a goal
		self.goal_sent = True
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
				Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

		# Start moving
		self.move_base.send_goal(goal)

		# Allow TurtleBot up to 60 seconds to complete task
		success = self.move_base.wait_for_result(rospy.Duration(60)) 

		state = self.move_base.get_state()
		result = False

		if success and state == GoalStatus.SUCCEEDED:
			# We made it!
			result = True
		else:
			self.move_base.cancel_goal()

		self.goal_sent = False

	# pos current position
	# marker marker recieved
	def approach(self, pos, marker):







