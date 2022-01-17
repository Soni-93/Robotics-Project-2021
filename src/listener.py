#!/usr/bin/env python
import rospy

from std_msgs.msg import String, Header
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image, PointField
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist


import sensor_msgs.point_cloud2 as pc2
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix
import tf2_ros as tf2

import numpy as np
import cv2

# Apparently ROS thinks Python is C :/
import ctypes
import struct

from move import MoveMe


pc_callback_count = 0
pc_callback_rate = 3 #5
goal_coords = None
goal_pose = None
has_set_goal = False
seq = 0

seen_map_msg = None

curr_pose = None
pub_map_seen = None

pub_cancel = None
pub_goal_found = None

def map_callback(msg):
	global seen_map_msg
	global pub_map_seen
	#print(dir(msg))

	# Maybe this will create a new topic?
	pub_map_seen = rospy.Publisher('map_seen', OccupancyGrid, queue_size=1)

	origin_pos = msg.info.origin.position
	res = msg.info.resolution
	map_width = msg.info.width
	map_height = msg.info.height



	print("Map Callback")
	print(origin_pos)
	print(map_height, map_width)

		
	data = np.array(msg.data)
	data[np.where((data < 0) | (data > 80))] = 100
	data[np.where(data!=100)] = 0

	msg.data = tuple(data)

	seen_map_msg = msg


	while pub_map_seen.get_num_connections() < 1:
		rospy.sleep(0.1)

	pub_map_seen.publish(msg)


def callback(data):
	print("-------------------------\n")
	print(data.encoding)
	w = data.width
	h = data.height
	print("Width: ", w, "Height: ", h)
	print(len(data.data)/(w*h), data.step)
	print(type(data.data[0]))
	print(data.data[0], data.data[1], data.data[2])

	rospy.sleep(1)

# I.e. decode data
def rgb_float_to_list(rgb_float):
	s = struct.pack('>f', rgb_float)
	i = struct.unpack('>l',s)[0]
	# you can get back the float value by the inverse operations
	pack = ctypes.c_uint32(i).value
	r = int ((pack & 0x00FF0000)>> 16)
	g = int ((pack & 0x0000FF00)>> 8)
	b = int (pack & 0x000000FF)
	return [r, g, b]

def callback_pc(ros_point_cloud):
	global pc_callback_count
	global pc_callback_rate
	# print("Hello")
	pc_callback_count = pc_callback_count +1
	if pc_callback_count >= pc_callback_rate:
		pc_callback_count = 0
		pc_proc(ros_point_cloud)

def get_3D_transf_matrix(transf_msg):
	#print("get_3D_transf_matrix_z")
	x_translate = transf_msg.transform.translation.x
	y_translate = transf_msg.transform.translation.y
	z_translate = transf_msg.transform.translation.z
	rot = transf_msg.transform.rotation
	euler_rot = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
	euler_mat = euler_matrix(euler_rot[0], euler_rot[1], euler_rot[2])

	transf_mat = euler_mat.copy()
	transf_mat[0][-1] = x_translate
	transf_mat[1][-1] = y_translate
	transf_mat[2][-1] = z_translate


	return transf_mat

def transf_sing(transf_mat, xyz_np):
    transf_xyz = np.matmul(transf_mat, np.append(xyz_np, 1))
    transf_xyz = np.delete(transf_xyz, -1)
    return transf_xyz

# xy - np.array
# transform_msg
def apply_transform(xyz_np, transf_msg):
	transf_mat = get_3D_transf_matrix(transf_msg)
	if len(xyz_np.shape) == 1:
		return transf_sing(transf_mat, xyz_np)
	pass

def pc_proc(ros_point_cloud):
	global goal_coords
	global has_set_goal
	global seen_map_msg

	print("-------------------------\n")
	width = ros_point_cloud.width
	height = ros_point_cloud.height



	tfBuffer = tf2.Buffer()
	listener = tf2.TransformListener(tfBuffer)

	tmp = list(pc2.read_points(ros_point_cloud))

	all_xyz = list(pc2.read_points(ros_point_cloud, field_names = ("x","y","z")))

	xyz_np = np.array(all_xyz)
	#print("xyz_np.shape - ", xyz_np.shape)
	xyz_np = xyz_np.reshape((height,width,3))

	all_rgb_float = list(pc2.read_points(ros_point_cloud, field_names = ("rgb")))

	all_rgb = [rgb_float_to_list(rgb_float[0]) for rgb_float in all_rgb_float]
	np_rgb = np.array(all_rgb)
	np_rgb = np_rgb.reshape((height,width,3))

	rgb_lower = [125,40,0]
	rgb_upper = [145,60,20]

	orange_indices = np.where(np.all((np_rgb > rgb_lower) & (np_rgb < rgb_upper), axis=-1))



	tot_pixels = width * height
	tol = 0.005 # 0.5%



	if orange_indices[0].shape[0]/float(tot_pixels) >= tol:
		xyz_obj = xyz_np[orange_indices[0], orange_indices[1]]
		obj_coords_mean = np.nanmean(xyz_obj.reshape((-1,3)), axis=0)



		transform_msg = None
		if not np.isnan(np.sum(obj_coords_mean)):
			print("Target found at " + str(obj_coords_mean) + " :)")
			# Cancel current path - /move_base/cancel
			pub_cancel.publish(GoalID())

			rospy.sleep(1)


			try:
				transform_msg = tfBuffer.lookup_transform("map","base_link", rospy.Time(0), rospy.Duration(0.1))
				print("TRANSFORM MESSAGE: \n" + str(transform_msg))
				transf_mat = get_3D_transf_matrix(transform_msg)

			except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
				print("transform_msg not found")
		else:
			print("Object found but coords cannot be determined")



		if not has_set_goal and not np.isnan(np.sum(obj_coords_mean)) and (transform_msg is not None):


			goal_coords_world = apply_transform(obj_coords_mean, transform_msg)



			pub_goal_found.publish(Twist())

			has_set_goal = True
			move_me = MoveMe()
			move_me.move_to_xyrot(goal_coords_world[0], goal_coords_world[1], 0)

	else:
		print("Target not found :(")

	if seen_map_msg is not None and False:
		print("MAP:")
		map_height = seen_map_msg.info.height
		map_width = seen_map_msg.info.width
		map_res = seen_map_msg.info.resolution
		map_origin_pos_obj = seen_map_msg.info.origin.position
		map_origin_xy = [map_origin_pos_obj.x, map_origin_pos_obj.y]

		print("map_res: ", map_res, str(map_origin_pos_obj))

		print("map_origin_xy:")
		print(map_origin_xy)



		map_data = np.array(seen_map_msg.data).reshape((map_height,map_width))

		xy_np = np.delete(xyz_np, 2,2)



		xy_transf = xy_np - map_origin_xy
		xy_transf = np.nan_to_num(xy_transf) # Replace NANs with zero

		xy_transf = (xy_transf/map_res).round().astype(int)

		xy_transf = xy_transf.reshape((-1,2))

		#print(xy_transf)
		print("map_data.shape: " + str(map_data.shape))
		print("xy_transf.shape: " + str(xy_transf.shape))

		# map_arr[index_arr[:,0], index_arr[:,1]]
		map_data[xy_transf[:,0], xy_transf[:,1]] = 100

		map_data = map_data.reshape(-1)

		seen_map_msg.data = tuple(map_data)

		pub_map_seen.publish(seen_map_msg)








def goal_cb(data):
	#print(dir(data))
	print(dir(data.pose))
	pos = data.pose.position
	orient = data.pose.orientation

	must_print = True
	if must_print:
		print("---------------------------")
		print("Goal data recieved")
		print(data)
		print("Position: " + str(pos))
		print("Orient Quaternion: " + str(orient))

		print("Orient Euler: " + str(euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])))
		print("---------------------------")


def move_to_xzrot(x,z,rot,pub_goal):
	global seq

	curr_pose = rospy.wait_for_message("/odom", Odometry)

	goal_msg = PoseStamped()



	odom_orient = curr_pose.twist.twist.angular
	odom_coords = curr_pose.twist.twist.linear

	y = odom_coords.y



	tmp_orient = quaternion_from_euler(odom_orient.x, odom_orient.y, rot)

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
	goal_msg.header.seq = seq
	seq = seq + 1





	pub_goal.publish(goal_msg)
	return 0



def main():
	global pub_cancel
	global pub_goal_found

	rospy.init_node('listener', anonymous=True)

	pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

	pub_goal_found = rospy.Publisher('/goal_found', Twist, queue_size=1)


	while pub_cancel.get_num_connections() < 1:
		rospy.sleep(0.1)

	rospy.Subscriber("/map", OccupancyGrid, map_callback)


	rospy.Subscriber("/camera/depth/points", PointCloud2, callback_pc)


	rospy.spin()


if __name__ == '__main__':
	main()
