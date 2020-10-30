#!/usr/bin/python
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy
import numpy as np

from nav_msgs.msg import Odometry

from util import rotateQuaternion, getHeading
from random import random


from time import time
ROOT_2_PI = math.sqrt(2*math.pi)

def find_weighted_random(sd): #use approximate normal to generate random number
	#takes standard deviation as input, returns a value from between -3 standard deviations
	#and 3 standard deviations chosen at random with proabability weighted using
	#expression for normal distribution
	values = np.arange(-3*sd,3.1*sd,sd/10.0)
	weights = [0]*len(values)
	sum_weights = 0
	for t in range(0,len(values)):
		weights[t] = (1/(ROOT_2_PI))*math.exp(-0.5*(values[t]/sd)**2)
		sum_weights = sum_weights + weights[t]
	z = random()*sum_weights
	for t in range(0,len(values)):
		if z < weights[t]:
			return values[t]
		z = z - weights[t]
	return 0 #base case to avoid infinite loop; should never run


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
	self.ODOM_ROTATION_NOISE = 0.05 # Odometry model rotation noise
	self.ODOM_TRANSLATION_NOISE = 0.25 # Odometry model x axis (forward) noise
	self.ODOM_DRIFT_NOISE = 0.25 # Odometry model y axis (side-to-side) noise

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

	particle_array.poses = [Odometry().pose.pose]*250

	for i in range(0,len(particle_array.poses)):
		xnoise = find_weighted_random(self.ODOM_TRANSLATION_NOISE)
		ynoise = find_weighted_random(self.ODOM_DRIFT_NOISE)
		znoise = find_weighted_random(self.ODOM_ROTATION_NOISE)

		new_particle = Odometry().pose.pose
		new_particle.position.x = initialpose.pose.pose.position.x + xnoise
		new_particle.position.y = initialpose.pose.pose.position.y + ynoise
		new_particle.orientation = rotateQuaternion(initialpose.pose.pose.orientation, znoise)
		particle_array.poses[i] = new_particle

	return particle_array
 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """

	new_particlecloud = PoseArray()
	new_particlecloud.poses = [Odometry().pose.pose]*len(self.particlecloud.poses)
	probs = [0]*len(self.particlecloud.poses)
	regions = [0]*len(self.particlecloud.poses)

	for i in range(0,len(self.particlecloud.poses)):

		probs[i] = self.sensor_model.get_weight(scan,self.particlecloud.poses[i])
		#this line causes crash, can replace with line below to continue
		#but naturally particle cloud wont actually update as it should
#		probs[i] = 0.4

	for i in range(0,len(self.particlecloud.poses)):
		benchmark = random()*sum(probs)
		j = 0
		k = 0
		while j < benchmark:
			j = j + probs[k]
			k = k + 1
		chosenpose = self.particlecloud.poses[k-1]
		new_particle = Odometry().pose.pose

		xnoise = find_weighted_random(self.ODOM_TRANSLATION_NOISE)
		ynoise = find_weighted_random(self.ODOM_DRIFT_NOISE)
		znoise = find_weighted_random(self.ODOM_ROTATION_NOISE)

		new_particle.position.x = chosenpose.position.x + xnoise
		new_particle.position.y = chosenpose.position.y + ynoise
		new_particle.orientation = rotateQuaternion(chosenpose.orientation, znoise)

		new_particlecloud.poses[i] = new_particle
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
	#this is very basic and needs a lot of improvement
	xsum = 0.0
	ysum = 0.0
	zsum = 0.0

	for i in self.particlecloud.poses:
		xsum = xsum + i.position.x
		ysum = ysum + i.position.y
		zsum = zsum + i.orientation.z

	new_pose = Odometry().pose.pose
	new_pose.position.x = xsum / len(self.particlecloud.poses)
	new_pose.position.y = ysum / len(self.particlecloud.poses)
	new_pose.orientation.z = zsum / len(self.particlecloud.poses)

	return new_pose








