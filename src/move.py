#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image, PointField
# tf2_msgs/TFMessage
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import PoseStamped

import numpy as np
import cv2

# Apparently ROS thinks Python is C :/
import ctypes
import struct

class MoveMe():
	def __init__(self):
		self.pc_callback_count = 0
		self.pc_callback_rate = 5
		self.goal_coords = None
		self.goal_pose = None
		self.has_set_goal = False
		self.seq = 0

		rospy.init_node('listener', anonymous=True)

		self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

		while self.pub_goal.get_num_connections() < 1:
			rospy.sleep(0.05)


	def move_to_xyrot(self,x,y,rot):
		curr_pose = rospy.wait_for_message("/odom", Odometry)

		goal_msg = PoseStamped()

		

		odom_orient = curr_pose.twist.twist.angular
		odom_coords = curr_pose.twist.twist.linear

		z = odom_coords.z

		#x,z = 1,1
		rot = odom_orient.z # temp

		tmp_orient = quaternion_from_euler(odom_orient.x, odom_orient.y, rot)
		#print(tmp_orient)
		goal_msg.pose.orientation.x = tmp_orient[0]
		goal_msg.pose.orientation.y = tmp_orient[1]
		goal_msg.pose.orientation.z = tmp_orient[2]
		goal_msg.pose.orientation.w = tmp_orient[3]
		goal_msg.pose.position.x = x
		goal_msg.pose.position.y = y
		goal_msg.pose.position.z = z

		goal_msg.header.frame_id = 'map'
		now = rospy.get_rostime()

		goal_msg.header.stamp.secs = now.secs
		goal_msg.header.stamp.nsecs = now.nsecs
		goal_msg.header.seq = self.seq
		self.seq = self.seq + 1
		#print(goal_msg)

		self.pub_goal.publish(goal_msg)
		return 0

def goal_cb(data):
	#print(dir(data))
	pos = data.pose.position
	orient = data.pose.orientation

	must_print = False
	if must_print:
		print("---------------------------")
		print("Goal data recieved")
		print(data)
		print("Position: " + str(pos))
		print("Orient Quaternion: " + str(orient))

		print("Orient Euler: " + str(euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])))
		print("---------------------------")




def main():



	move_me = MoveMe()

	x,y,rot = input("Where do you wanna go? Format: x,y,rot:   ")

	move_me.move_to_xyrot(x,y,rot)


	rospy.spin()





if __name__ == '__main__':
	main()
