#!/usr/bin/env python

import rospy
import tf
import numpy as np
import time
from control import lqr
from std_msgs.msg import Float64, Float32
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped

# robot description
# d = 0.3 %0.29;
# l = 0.4 %0.2907;
# r = 0.2 %0.07;
# mb = 10 %4;
# mw = 1 %0.1;
# J = 0.02 %0.000735;
# K = 0.01833 %0.00039;
# I1 = 0.15 %0.015625;
# I2 = 0.6083 %0.0118625;
# I3 = 0.6083 %0.0118625;
# calpha = 0.0;
# g = 9.81;

# this code follows standerd derivation for 2 wheel inverted cart pendulum (very accurate model) --> 
# Dynamic Modeling of a Two-wheeled Inverted Pendulum Balancing Mobile Robot 
# by - Sangtae Kim and SangJoo Kwon
# Please read accurate.pdf for a better explanation on obtaining standard manipulator equations 
# M(q)q'' + C(q,q')q' + Dq' + G(q) = B*t

class sb_bot():
	# initializing state variables and required publishers
	def __init__(self):
		rospy.init_node('sb_LQR')
		self.bot_location = [0.0, 0.0, 0.0]
		self.bot_velocity = [0.0, 0.0, 0.0]
		self.bot_acceleration = [0.0, 0.0, 0.0]

		self.bot_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
		self.bot_orientation_euler = [0.0, 0.0, 0.0]
		self.bot_angular_velocity = [0.0, 0.0, 0.0]

		self.set_angle = 0.0
		# state varibales: [x', th', psi', x, th, psi]
		self.set_point = np.array([[0.0], [0.0], [0.0], [2.0], [0.0], [0.0]])

		# A B matrix for state space equation 
		# you can find this A B matrices from state_space_inv_pend.m file in this folder
		self.A = np.array([[-0.1184, 0.0237, 0, 0, -12.3514, 0], 
						   [0.2597, -0.0519, 0, 0, 40.142, 0], 
						   [0, 0, -0.0158, 0, 0, 0], 
						   [1, 0, 0, 0, 0, 0], 
						   [0, 1, 0, 0, 0, 0], 
						   [0, 0, 1, 0, 0, 0]])

		self.B = np.array([[1.1839, 1.1839 ], 
						   [-2.5968, -2.5968], 
						   [-1.0527, 1.0527], 
						   [0.0 , 0.0], 
						   [0, 0      ], 
						   [0.0, 0.0 ]])

		# Q and R matrix to penalize diffrent state errors and acctuator capcity 
		self.Q = np.array([[10, 0, 0, 0, 0, 0], 
						   [0, 10, 0, 0, 0, 0], 
						   [0, 0, 0.1, 0, 0, 0], 
						   [0, 0, 0, 3, 0, 0], 
						   [0, 0, 0, 0, 1000, 0], 
						   [0, 0, 0, 0, 0, 5]])

		self.R = np.array([[1, 0],
						   [0, 1]])

		# turque control
		self.left_wheel_vel = rospy.Publisher('/balance_bot/left_wheel_controller/command', Float64, queue_size=10)
		self.right_wheel_vel = rospy.Publisher('/balance_bot/right_wheel_controller/command', Float64, queue_size=10)

		rospy.Subscriber('/imu', Imu, self.imu_callback)
		rospy.Subscriber('/gps', NavSatFix, self.gps_callback)
		rospy.Subscriber('/gps_velocity', Vector3Stamped, self.gps_vel_callback)


	# function to convert quaternion to euler to find orientation, extracting angular and linear velocity
	def imu_callback(self, data):
		self.bot_orientation_quaternion[0] = data.orientation.x
		self.bot_orientation_quaternion[1] = data.orientation.y
		self.bot_orientation_quaternion[2] = data.orientation.z
		self.bot_orientation_quaternion[3] = data.orientation.w

		(self.bot_orientation_euler[0], self.bot_orientation_euler[1], self.bot_orientation_euler[2]) = tf.transformations.euler_from_quaternion(
            [self.bot_orientation_quaternion[0], self.bot_orientation_quaternion[1], self.bot_orientation_quaternion[2], self.bot_orientation_quaternion[3]])

		self.bot_angular_velocity[0] = data.angular_velocity.x
		self.bot_angular_velocity[1] = data.angular_velocity.y
		self.bot_angular_velocity[2] = data.angular_velocity.z

		self.bot_acceleration[0] = -data.linear_acceleration.x
		self.bot_acceleration[1] = -data.linear_acceleration.y
		self.bot_acceleration[2] = -data.linear_acceleration.z

	# finding out x y positions
	def gps_callback(self, data):
		self.bot_location[0] = (data.latitude - 49.9)/0.0000091149
		self.bot_location[1] = -(data.longitude - 8.9)/0.0000138728
		self.bot_location[2] = data.altitude
		# print("location ", self.bot_location)

	def gps_vel_callback(self, data):
		self.bot_velocity[0] = data.vector.x
		self.bot_velocity[1] = data.vector.y
		self.bot_velocity[2] = data.vector.z
		# print("bot_velocity ", self.bot_velocity)

	def LQR(self):
		# finding K gains from lqr function
		K, S, e = lqr(self.A, self.B, self.Q, self.R)
		# finding out error from previous states
		error_matrix = np.array([[self.set_point[0, 0] - self.bot_velocity[0]],
								 [self.set_point[1, 0] - self.bot_angular_velocity[1]],
								 [self.set_point[2, 0] - self.bot_angular_velocity[2]],
								 [self.set_point[3, 0] - self.bot_location[0]],
								 [self.set_point[4, 0] - self.bot_orientation_euler[1]],
								 [self.set_point[5, 0] - self.bot_orientation_euler[2]]])
		# u = -K*error_matrix
		output = np.dot(K, error_matrix)

		
		self.left_wheel_vel.publish(output[0])
		self.right_wheel_vel.publish(output[1])


def main():
	print("main")
	t = time.time()
	while True:
		bot.LQR()
		time.sleep(0.01)


if __name__ == '__main__':

    # making bot object of sb_bot class
    bot = sb_bot()

    # pause of 1 sec
    t = time.time()
    while time.time() - t < 1:
        pass

    while not rospy.is_shutdown():
        main()
        break
