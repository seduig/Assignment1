#!/usr/bin/python
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from sensor_msgs.msg import LaserScan
from pf_base import PFLocaliserBase
import math
import rospy
import numpy as np

from nav_msgs.msg import Odometry

from util import rotateQuaternion, getHeading, multiply_quaternions
from random import random, gauss
import sensor_model

from time import time, sleep

noise = 7

class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
	self.ODOM_ROTATION_NOISE = 0.65 # Odometry model rotation noise
	self.ODOM_TRANSLATION_NOISE = 0.4 # Odometry model x axis (forward) noise
	self.ODOM_DRIFT_NOISE = 0.4 # Odometry model y axis (side-to-side) noise

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """

	particle_array = PoseArray()
	base = initialpose.pose.pose
	for i in range(0,750):
		particle_array.poses.append(Odometry().pose.pose)
		particle_array.poses[i].position.x = gauss(base.position.x, 0.8)
		particle_array.poses[i].position.y = gauss(base.position.y, 0.8)
		particle_array.poses[i].orientation = rotateQuaternion(base.orientation, gauss(0,1.3))

	return particle_array
 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
	ranges_list = list(scan.ranges)
	for i in range(0,len(ranges_list)):
		if math.isnan(ranges_list[i]):
			ranges_list[i] = -1.0
	scan.ranges = tuple(ranges_list)
	global noise

	probs = []
	for i in range(0,len(self.particlecloud.poses)):
		probs.append(self.sensor_model.get_weight(scan,self.particlecloud.poses[i])**2)

	if sum(probs)/len(probs) < 28 and max(probs) < 31:

		for i in range(0,len(self.particlecloud.poses)):
			if probs[i] < 19:
				self.particlecloud.poses[i].position.x = gauss(self.estimatedpose.pose.pose.position.x,noise)
				self.particlecloud.poses[i].position.y = gauss(self.estimatedpose.pose.pose.position.y,noise)
#				self.particlecloud.poses[i].position.x = self.estimatedpose.pose.pose.position.x + noise*(random()*2-1)
#				self.particlecloud.poses[i].position.y = self.estimatedpose.pose.pose.position.y + noise*(random()*2-1)
#				self.particlecloud.poses[i].position.x = self.sensor_model.map_width/2.0
#				self.particlecloud.poses[i].position.y = self.sensor_model.map_height/2.0
#				self.particlecloud.poses[i].position.x = random()*self.sensor_model.map_width
#				self.particlecloud.poses[i].position.y = random()*self.sensor_model.map_height
				self.particlecloud.poses[i].orientation.z = 0.0
				self.particlecloud.poses[i].orientation.w = 1.0
				self.particlecloud.poses[i].orientation = rotateQuaternion(self.particlecloud.poses[i].orientation, math.pi*(random()*2 - 1))
		noise = noise + 1
	elif sum(probs)/len(probs) < 36 and max(probs) < 42:
		for i in range(0,len(self.particlecloud.poses)):
			if probs[i] < 11:
				self.particlecloud.poses[i].position.x = gauss(self.estimatedpose.pose.pose.position.x,noise)
				self.particlecloud.poses[i].position.y = gauss(self.estimatedpose.pose.pose.position.y,noise)
#				self.particlecloud.poses[i].position.x = self.estimatedpose.pose.pose.position.x + noise*(random()*2-1)
#				self.particlecloud.poses[i].position.y = self.estimatedpose.pose.pose.position.y + noise*(random()*2-1)
#				self.particlecloud.poses[i].position.x = self.sensor_model.map_width/2.0
#				self.particlecloud.poses[i].position.y = self.sensor_model.map_height/2.0
#				self.particlecloud.poses[i].position.x = random()*self.sensor_model.map_width
#				self.particlecloud.poses[i].position.y = random()*self.sensor_model.map_height
				self.particlecloud.poses[i].orientation.z = 0.0
				self.particlecloud.poses[i].orientation.w = 1.0
				self.particlecloud.poses[i].orientation = rotateQuaternion(self.particlecloud.poses[i].orientation, math.pi*(random()*2 - 1))
		noise = noise + 1
	else:

		noise = 7
		new_particlecloud = PoseArray()
		for i in range(0,len(self.particlecloud.poses)):
			benchmark = random()*sum(probs)
			j = 0
			k = 0
			while j < benchmark:
				j = j + probs[k]
				k = k + 1
			chosenpose = self.particlecloud.poses[k-1]
			new_particlecloud.poses.append(Odometry().pose.pose)
			new_particlecloud.poses[i].position.x = gauss(chosenpose.position.x, 0.04)
			new_particlecloud.poses[i].position.y = gauss(chosenpose.position.y, 0.04)
			new_particlecloud.poses[i].orientation = rotateQuaternion(chosenpose.orientation, gauss(0, 0.03))
		self.particlecloud = new_particlecloud



    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """

	xsum = 0.0
	ysum = 0.0
	sinsum = 0.0
	cossum = 0.0

	xvals = []
	yvals = []
	for i in self.particlecloud.poses:
		xvals.append(i.position.x)
		yvals.append(i.position.y)

	xvals.sort()
	yvals.sort()

	valids = 0


	for i in self.particlecloud.poses:
		if i.position.x in xvals[len(xvals)/4:3*len(xvals)/4] and i.position.y in yvals[len(yvals)/4:3*len(yvals)/4]:
			valids = valids + 1
			xsum = xsum + i.position.x
			ysum = ysum + i.position.y
			sinsum = sinsum + 2*i.orientation.z*i.orientation.w
			cossum = cossum + i.orientation.w**2 - i.orientation.z**2

	new_pose = Odometry().pose.pose
	new_pose.position.x = xsum / valids
	new_pose.position.y = ysum / valids
	new_pose.orientation.w = 1.0
	new_pose.orientation = rotateQuaternion(new_pose.orientation,math.atan2(sinsum,cossum))

	return new_pose








