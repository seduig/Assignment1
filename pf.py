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

	particle_array = PoseArray() #start with blank PoseArray for the particle cloud
	base = initialpose.pose.pose
	for i in range(0,750): #we will generate 750 particles
		particle_array.poses.append(Odometry().pose.pose) #add new particle
		#give this particle the x and y position of the initial pose plus a little gaussian noise for each
		particle_array.poses[i].position.x = gauss(base.position.x, 0.8) 
		particle_array.poses[i].position.y = gauss(base.position.y, 0.8)
		#and give it the orientation of the initial pose, again with some gaussian noise
		particle_array.poses[i].orientation = rotateQuaternion(base.orientation, gauss(0,1.3))

	return particle_array
 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
	ranges_list = list(scan.ranges) #need to iterate over scan.ranges to edit invalid values, this cannot be done on tuples so convert
	for i in range(0,len(ranges_list)):
		if math.isnan(ranges_list[i]): #check if each value is 'nan', which turns up occasionally on real data scans
			ranges_list[i] = -1.0 #if it is, replace it with -1.0 so the program can continue without error
	scan.ranges = tuple(ranges_list) #replace scan.ranges with altered version
	global noise

	probs = []
	for i in range(0,len(self.particlecloud.poses)): #find weights of all particles
		probs.append(self.sensor_model.get_weight(scan,self.particlecloud.poses[i])**2)
		#weights are squared to increase importance of high weight particles

	if sum(probs)/len(probs) < 28 and max(probs) < 31: #if robot kidnapped:

		for i in range(0,len(self.particlecloud.poses)):
			if probs[i] < 19: #replace all low weight particles
				#with random gaussian positions chosen using the most recent estimated pose as a center
				self.particlecloud.poses[i].position.x = gauss(self.estimatedpose.pose.pose.position.x,5)
				self.particlecloud.poses[i].position.y = gauss(self.estimatedpose.pose.pose.position.y,5)
				#and choose a random orientation for each one
				self.particlecloud.poses[i].orientation.z = 0.0
				self.particlecloud.poses[i].orientation.w = 1.0
				self.particlecloud.poses[i].orientation = rotateQuaternion(self.particlecloud.poses[i].orientation, math.pi*(random()*2 - 1))
		 		
	elif sum(probs)/len(probs) < 36 and max(probs) < 42: #if robot *maybe* kidnapped:
		for i in range(0,len(self.particlecloud.poses)):
			if probs[i] < 11: #replace only the *very* low weight particles in the same way as above
				self.particlecloud.poses[i].position.x = gauss(self.estimatedpose.pose.pose.position.x,5)
				self.particlecloud.poses[i].position.y = gauss(self.estimatedpose.pose.pose.position.y,5)
				self.particlecloud.poses[i].orientation.z = 0.0
				self.particlecloud.poses[i].orientation.w = 1.0
				self.particlecloud.poses[i].orientation = rotateQuaternion(self.particlecloud.poses[i].orientation, math.pi*(random()*2 - 1))

	else: #if robot not thought to be kidnapped:
		new_particlecloud = PoseArray() #start with a blank PoseArray for the new cloud
		for i in range(0,len(self.particlecloud.poses)): #stochastic universal sampling
			benchmark = random()*sum(probs) #choose random number between 0 and sum of weights
			j = 0 #dummy variable to represent regions; each particle has a region proportional to it's weight
			k = 0 #dummy variable to count
			while j < benchmark: #while we have not yet reached the region containing the random value:
				j = j + probs[k] #iterate to the next region
				k = k + 1 #consider next region
			#once we have reached the region containing the random value, choose the particle this region represented
			chosenpose = self.particlecloud.poses[k-1]
			new_particlecloud.poses.append(Odometry().pose.pose)
			new_particlecloud.poses[i].position.x = gauss(chosenpose.position.x, 0.04) #add a little noise
			new_particlecloud.poses[i].position.y = gauss(chosenpose.position.y, 0.04)
			new_particlecloud.poses[i].orientation = rotateQuaternion(chosenpose.orientation, gauss(0, 0.03))
		self.particlecloud = new_particlecloud #replace cloud with new cloud



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

	xsum = 0.0 #sum of valid x positions for finding average
	ysum = 0.0 #same for y
	sinsum = 0.0 #sum of the sin's of angles found to be valid
	cossum = 0.0 #same for cos's

	xvals = [] #list containing all x positions
	yvals = [] #same for y
	for i in self.particlecloud.poses: #add all x and y positions to these lists
		xvals.append(i.position.x)
		yvals.append(i.position.y)

	xvals.sort() #sort them so we can discard particles with positions in bottom and top quartiles
	yvals.sort()

	valids = 0 #amount of undiscarded particles


	for i in self.particlecloud.poses:
		if i.position.x in xvals[len(xvals)/4:3*len(xvals)/4] and i.position.y in yvals[len(yvals)/4:3*len(yvals)/4]:
		#if particle's x and y position are each not in the bottom or top quartile, we consider this particle valid
			valids = valids + 1
			xsum = xsum + i.position.x
			ysum = ysum + i.position.y
			#for an angle d representated by a quaternion x,y,z,w:
			#z = sin(d/2) and w = cos(d/2) (easy to check), so:
			#sin(d) = sin(2*d/2) = 2*sin(d/2)*cos(d/2) = 2*z*w
			sinsum = sinsum + 2*i.orientation.z*i.orientation.w
			#and cos(d) = cos(2*d/2) = cos(d/2)**2 - sin(d/2)**2 = w**2 - z**2
			cossum = cossum + i.orientation.w**2 - i.orientation.z**2

	new_pose = Odometry().pose.pose
	new_pose.position.x = xsum / valids #choose x position of average of valid particles
	new_pose.position.y = ysum / valids #same for y
	new_pose.orientation.w = 1.0
	#atan2 used to find average of angles
	new_pose.orientation = rotateQuaternion(new_pose.orientation,math.atan2(sinsum,cossum))

	return new_pose








