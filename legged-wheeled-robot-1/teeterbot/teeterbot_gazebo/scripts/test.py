#!/usr/bin/env python

import rospy
import tf
import tf.transformations
#import numpy as np
import time
#from control import *
#from std_msgs.msg import Float64, Float32
from sensor_msgs.msg import Imu
#from sensor_msgs.msg import NavSatFix
#from geometry_msgs.msg import Vector3Stamped

#bot_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
#bot_orientation_euler = [0.0, 0.0, 0.0]

def imu_callback(data):
		q_x = data.orientation.x
		q_y = data.orientation.y
		q_z = data.orientation.z
		q_w = data.orientation.w
		#print(self.bot_orientation_quaternion)
		eu = tf.transformations.euler_from_quaternion([q_x,q_y,q_z,q_w],axes='sxyz')
		#print eu
		print(eu)


rospy.init_node('test_node')
#Rx = rotation_matrix(alpha, xaxis)
#Ry = rotation_matrix(beta, yaxis)
#Rz = rotation_matrix(gamma, zaxis)
#R = concatenate_matrices(Rx, Ry, Rz)
#euler = euler_from_matrix(R, 'rxyz')
#print (euler)

rospy.Subscriber('/imu', Imu, imu_callback)
rospy.spin()
