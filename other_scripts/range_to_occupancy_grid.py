#!/usr/bin/env python

# OccupancyGrid publisher
# author: Ricmirodo Giubilato
# mail: ricmirodo.giubilato@gmail.com
# year: 2017
# ============================================
# subscribed topics:
#	/miro/scan - contains LaserScan messages
# published topics:
#   /miro/map - contains OccupancyGrid messages
# listened tf:
#   /miro_base_link to /world
#	/sonar_link to /miro_base_link
#   /map to /world
# ============================================

import rospy
import numpy
import math
import tf
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

# initialize node
rospy.init_node('range_to_occupancy_grid_node')

# listener of transforms between the miro_base_link and the world frame
miro_pose = tf.TransformListener()

# Initialize occupancy grid message
map_msg = OccupancyGrid()
map_msg.header.frame_id = 'map'
resolution = 0.01  # 1cm
width = 200  # 100x1cm = 1m
height = 200

# Initialize miro pose relative to world
x_miro = 0.0
y_miro = 0.0

# square size of the miro footprint [m]
footprint = 0.25  # 25cm

# Map update rate (defaulted to 10 Hz)
rate = 10.0

# Range data
miro_range = 0.0

def set_free_cells(grid, position, size):
	# set free the cells occupied by the miro
	# grid:				ndarray [width,height]
	# position:			[x y] pose of the miro
	# size: 			r     radius of the footprint
	global resolution

	off_x = position[1] // resolution + width  // 2
	off_y = position[0] // resolution + height // 2

	# set the roi to 1: known free positions
	for i in range(-size//2, size//2):
		for j in range(-size//2, size//2):
			if not (0 < int(i + off_x) < grid.shape[0] and 0 < int(j + off_y) < grid.shape[1]):
				continue
			grid[int(i + off_x), int(j + off_y)] = 1

def set_obstacle(grid, position, orientation, position_sonar, quaternion_sonar, miro_range):
	# set the occupied cells when detecting an obstacle
	# grid:				ndarray [width,height]
	# position:			[x y] pose of the miro
	# orientation:      quaternion, orientation of the miro
	global resolution

	off_x = position[1] // resolution + width  // 2
	off_y = position[0] // resolution + height // 2
	# off_x = position_sonar[1] // resolution + width  // 2
	# off_y = position_sonar[0] // resolution + height // 2
	

	# print orientation
	# print quaternion_sonar
	# print tf.transformations.quaternion_multiply(quaternion_sonar, orientation)*-1
	# euler = tf.transformations.euler_from_quaternion(orientation)
	# euler = tf.transformations.euler_from_quaternion(quaternion_sonar)
	euler = tf.transformations.euler_from_quaternion(tf.transformations.quaternion_multiply(quaternion_sonar, orientation)*-1)

	# do not update is the range is 0 (most likely to be a wrong measurement) or higher than 30 (low accuracy above the maximal specified range)
	if not (miro_range == 0.0 or miro_range > 0.30):

		rotMatrix = numpy.array([[numpy.cos(euler[2]),   numpy.sin(euler[2])],
			                     [-numpy.sin(euler[2]),  numpy.cos(euler[2])]])
		
		####
		# tf lookup for position_sonar is not implemented 
		# just add here
		# position_sonar[0] = 0.15  # sonar is located aprox. 15 cm in front of the base 
		####

		obstacle = numpy.dot(rotMatrix,numpy.array([0, (miro_range + position_sonar[0]) // resolution])) + numpy.array([off_x,off_y])
		# obstacle = numpy.dot(rotMatrix,numpy.array([position_sonar[1], (miro_range + position_sonar[0]) // resolution])) + numpy.array([off_x,off_y])
		# obstacle = numpy.dot(rotMatrix,numpy.array([0, (miro_range) // resolution])) + numpy.array([off_x,off_y])

		print grid.shape
		print obstacle

		if not (0 < int(obstacle[0]) < grid.shape[0]-1 and 0 < int(obstacle[1]) < grid.shape[1]-1):
			return

		rospy.loginfo("FOUND OBSTACLE AT: x:%f y:%f", obstacle[0], obstacle[1])

		# set probability of occupancy to 100 and neighbour cells to 100
		MAX = 100
		OBSTACLE = 50  # probability of obstacle at pos
		NEIGHBOR = 25  # probability of obstacle at neighboring pos
		SET_FREE = 100    # 1 if free fields (robot was there already) should not be updated, 2 if free fields should be updated (the robot can come really close to the obstacle and can then delete the boundary), 100 everything gets updated
		grid[int(obstacle[0]), int(obstacle[1])] = min(grid[int(obstacle[0]), int(obstacle[1])] + int(OBSTACLE), MAX)
		if  grid[int(obstacle[0]+1), int(obstacle[1])]   < int(SET_FREE):
			grid[int(obstacle[0]+1), int(obstacle[1])]   = min(grid[int(obstacle[0]+1), int(obstacle[1])] + int(NEIGHBOR), MAX)
		if  grid[int(obstacle[0]), 	 int(obstacle[1]+1)] < int(SET_FREE):
			grid[int(obstacle[0]),   int(obstacle[1]+1)] = min(grid[int(obstacle[0]), int(obstacle[1]+1)] + int(NEIGHBOR), MAX)
		if  grid[int(obstacle[0]-1), int(obstacle[1])]   < int(SET_FREE):
			grid[int(obstacle[0]-1), int(obstacle[1])]   = min(grid[int(obstacle[0]-1), int(obstacle[1])] + int(NEIGHBOR), MAX)
		if  grid[int(obstacle[0]),   int(obstacle[1]-1)] < int(SET_FREE):
			grid[int(obstacle[0]),   int(obstacle[1]-1)] = min(grid[int(obstacle[0]), int(obstacle[1]-1)] + int(NEIGHBOR), MAX)


        ###  no idea??
		# t = 0.5
		# i = 1
		# free_cell = numpy.dot(rotMatrix,numpy.array([0, t*i])) + numpy.array([off_x,off_y])
		# if not (0 < int(free_cell[0]) < grid.shape[0]-1 and 0 < int(free_cell[1]) < grid.shape[1]-1):
		# 	return
		# while grid[int(free_cell[0]), int(free_cell[1])] < int(1):
		# 	grid[int(free_cell[0]), int(free_cell[1])] = int(0)
		# 	free_cell = numpy.dot(rotMatrix,numpy.array([0, t*i])) + numpy.array([off_x,off_y])
		# 	i = i+1;

def callback_range(msg):
	# callback range
	global miro_range
	miro_range = msg.range


# Subscribers
range_sub = rospy.Subscriber("/range", Range, callback_range)

# Publishers
occ_pub = rospy.Publisher("/map", OccupancyGrid, queue_size = 10)


# main function
if __name__ == '__main__':

	# set grid parameters
	if rospy.has_param("occupancy_rate"):
		rate = rospy.get_param("occupancy_rate")

	if rospy.has_param("grid_resolution"):
		resolution = rospy.get_param("grid_resolution")

	if rospy.has_param("grid_width"):
		width = rospy.get_param("grid_width")

	if rospy.has_param("grid_height"):
		height = rospy.get_param("grid_height")

	# fill map_msg with the parameters from launchfile
	map_msg.info.resolution = resolution
	map_msg.info.width = width
	map_msg.info.height = height
	map_msg.data = range(width*height)

	# initialize grid with -1 (unknown)
	grid = numpy.ndarray((width, height), buffer=numpy.zeros((width, height), dtype=numpy.int),
	         dtype=numpy.int)
	grid.fill(int(-1))

	# set map origin [meters]
	map_msg.info.origin.position.x = - width // 2 * resolution
	map_msg.info.origin.position.y = - height // 2 * resolution

	loop_rate = rospy.Rate(rate)

	while not rospy.is_shutdown():

		try:
			t = miro_pose.getLatestCommonTime("/base_link", "/odom")
			position, quaternion = miro_pose.lookupTransform("/odom", "/base_link", t)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "error1"
			continue

		try:
			t = miro_pose.getLatestCommonTime("/base_link", "/sonar_link")
			position_sonar, quaternion_sonar = miro_pose.lookupTransform("/base_link", "/sonar_link", t)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			print e
			print "error2"
			continue

		# write 0 (null obstacle probability) to the free areas in grid
		set_free_cells(grid, position, int(footprint//resolution))

		# write p>0 (non-null obstacle probability) to the occupied areas in grid
		set_obstacle(grid, position, quaternion, position_sonar, quaternion_sonar, miro_range)
		# set_obstacle(grid, position, quaternion, position, quaternion, miro_range)


		# stamp current ros time to the message
		map_msg.header.stamp = rospy.Time.now()

		# build ros map message and publish
		for i in range(width*height):
			map_msg.data[i] = grid.flat[i]
		occ_pub.publish(map_msg)

		br = tf.TransformBroadcaster()
		br.sendTransform((0, 0, 0), 
        	tf.transformations.quaternion_from_euler(0, 0, 0),
        	rospy.Time.now(),
        	"map",
        	"odom")

		loop_rate.sleep()

	## dump grid as csv
	numpy.savetxt("map.csv", grid, delimiter=',')