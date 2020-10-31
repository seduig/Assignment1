#!/usr/bin/python
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy
import numpy as np

from nav_msgs.msg import Odometry

from util import rotateQuaternion, getHeading, multiply_quaternions
from random import random


from time import time
ROOT_2_PI = math.sqrt(2*math.pi)

def find_weighted_random(sd): #use approximate normal to generate random number
	#takes standard deviation as input, returns a value from between -3 standard deviations
	#and 3 standard deviations chosen at random with proabability weighted using
	#expression for normal distribution
	values = np.arange(-3*sd,3*sd,sd/10.0)
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
	self.ODOM_TRANSLATION_NOISE = 0.2 # Odometry model x axis (forward) noise
	self.ODOM_DRIFT_NOISE = 0.2 # Odometry model y axis (side-to-side) noise

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
#		print(probs[i])
#	print(probs)
#	print(float(sum(probs))/float(len(probs)))
	for i in range(0,len(self.particlecloud.poses)):
		benchmark = random()*sum(probs)
		j = 0
		k = 0
		while j < benchmark:
			j = j + probs[k]
			k = k + 1
		chosenpose = self.particlecloud.poses[k-1]
#		print(k-1)
#		print(probs[k-1])
		new_particle = Odometry().pose.pose

		xnoise = find_weighted_random(0.01)
		ynoise = find_weighted_random(0.01)
		znoise = find_weighted_random(0.01)

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

	xsum = 0.0
	ysum = 0.0
	sinsum = 0.0
	cossum = 0.0

	xmin = min(i.position.x for i in self.particlecloud.poses)
	xmax = max(i.position.x for i in self.particlecloud.poses)
	xrng = xmax - xmin
	ymin = min(i.position.y for i in self.particlecloud.poses)
	ymax = max(i.position.y for i in self.particlecloud.poses)
	yrng = ymax - ymin
#	zmin = min(i.orientation.z for i in self.particlecloud.poses)
#	zmax = max(i.orientation.z for i in self.particlecloud.poses)
#	zrng = zmax - zmin

	valids = 0

	for i in self.particlecloud.poses:
		if i.position.x > xmin + xrng/3 and i.position.x < xmax - xrng/3 and i.position.y > ymin + yrng/3 and i.position.y < ymax - yrng/3:
			valids = valids + 1
			xsum = xsum + i.position.x
			ysum = ysum + i.position.y
			sinsum = sinsum + 2*i.orientation.z*i.orientation.w
			cossum = cossum + i.orientation.w**2 - i.orientation.z**2
#	print(len(valids))
	new_pose = Odometry().pose.pose
	new_pose.position.x = xsum / valids
	new_pose.position.y = ysum / valids
	new_pose.orientation.w = 1.0
	new_pose.orientation = rotateQuaternion(new_pose.orientation,math.atan2(sinsum,cossum))


#	new_pose.orientation.w = wsum / len(valids)
#	print(new_pose.orientation.z,new_pose.orientation.w, math.cos(math.asin(new_pose.orientation.z)))
#	print(new_pose)

#	dummy1 = Odometry().pose.pose.orientation
#	dummy2 = Quaternion()
#	dummy1.w = 1.0
#	dummy2.w = 1.0
#	dummy1.z = 0.9
#	dummy1.w = math.cos(math.asin(0.9))
#	dummy2 = Odometry().pose.pose.orientation
#	dummy2.z = 0.9
#	dummy2.w = math.cos(math.asin(0.9))
#	print(dummy1, dummy2, multiply_quaternions(dummy1,dummy2))

#	print(rotateQuaternion(dummy1, -math.pi/2))
#	print(getHeading(rotateQuaternion(dummy1, -math.pi/2)))
#	print(new_pose)
	return new_pose








