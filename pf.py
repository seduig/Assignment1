#!/usr/bin/python
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy
import numpy as np

from nav_msgs.msg import Odometry

from util import rotateQuaternion, getHeading, multiply_quaternions
from random import random, gauss


from time import time, sleep

noise = 2

class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
	self.ODOM_ROTATION_NOISE = 0.01 # Odometry model rotation noise
	self.ODOM_TRANSLATION_NOISE = 0.02 # Odometry model x axis (forward) noise
	self.ODOM_DRIFT_NOISE = 0.02 # Odometry model y axis (side-to-side) noise

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
	for i in range(0,1000):
		particle_array.poses.append(Odometry().pose.pose)
		particle_array.poses[i].position.x = gauss(base.position.x, 0.4)
		particle_array.poses[i].position.y = gauss(base.position.y, 0.4)
		particle_array.poses[i].orientation = rotateQuaternion(base.orientation, gauss(0,0.1))

	return particle_array
 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """

	global noise

	probs = [0]*len(self.particlecloud.poses)
	x = time()
	for i in range(0,len(self.particlecloud.poses)):
		probs[i] = self.sensor_model.get_weight(scan,self.particlecloud.poses[i])
	print('total prob time: ', time() - x)
	print(min(probs),max(probs),sum(probs)/len(probs))


	if sum(probs)/len(probs) < 1.5 and max(probs) < 60:
		print('low max and average likelihood. robot assumed kidnapped')
		print(noise)

		for i in range(0,len(self.particlecloud.poses)):
			if probs[i] < 3.5:
				self.particlecloud.poses[i].position.x = gauss(self.estimatedpose.pose.pose.position.x,noise)
				self.particlecloud.poses[i].position.y = gauss(self.estimatedpose.pose.pose.position.y,noise)
#			new_particle.position.x = random()*self.sensor_model.map_width
#			new_particle.position.y = random()*self.sensor_model.map_height
				self.particlecloud.poses[i].orientation.z = 0.0
				self.particlecloud.poses[i].orientation.w = 1.0
				self.particlecloud.poses[i].orientation = rotateQuaternion(self.particlecloud.poses[i].orientation, math.pi*(random()*2 - 1))
		noise = noise + 1

	else:

		noise = 2
#		usable = []
#		u_probs = []
#		for i in range(0,len(self.particlecloud.poses)):
#			if probs[i] > 4:
#				usable.append(self.particlecloud.poses[i])
#				u_probs.append(probs[i])
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
			new_particlecloud.poses[i].position.x = gauss(chosenpose.position.x, self.ODOM_TRANSLATION_NOISE)
			new_particlecloud.poses[i].position.y = gauss(chosenpose.position.y, self.ODOM_DRIFT_NOISE)
			new_particlecloud.poses[i].orientation = rotateQuaternion(chosenpose.orientation, gauss(0, self.ODOM_ROTATION_NOISE))	
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

#	xmin = min(i.position.x for i in self.particlecloud.poses)
#	xmax = max(i.position.x for i in self.particlecloud.poses)
#	xrng = xmax - xmin
#	ymin = min(i.position.y for i in self.particlecloud.poses)
#	ymax = max(i.position.y for i in self.particlecloud.poses)
#	yrng = ymax - ymin

	xvals = []
	yvals = []
	for i in self.particlecloud.poses:
		xvals.append(i.position.x)
		yvals.append(i.position.y)
#	print(xvals)
	xvals.sort()
	yvals.sort()

#	zmin = min(i.orientation.z for i in self.particlecloud.poses)
#	zmax = max(i.orientation.z for i in self.particlecloud.poses)
#	zrng = zmax - zmin

	valids = 0


	for i in self.particlecloud.poses:
#		if i.position.x > xmin + xrng/3 and i.position.x < xmax - xrng/3 and i.position.y > ymin + yrng/3 and i.position.y < ymax - yrng/3: #fix
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

#	else:
		


	return new_pose








