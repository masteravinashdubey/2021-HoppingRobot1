#!/usr/bin/env python

# inverted pendulum model to balance the robot 
#      ________
#     |        |
#     |        |
#     |   /    |      --->
#     |__/_____|
#       / /
#      / /
#     / /	ABHIMANYU
#     \ \
#      \ \
#       \ \
#        \ \
#         \(O)
#         (O)

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

class lwr_bot():
	def __init__(self):
		rospy.init_node('lwr_wheel_lqr')

		self.bot_location = [0.0, 0.0, 0.0]
		self.bot_velocity = [0.0, 0.0, 0.0]
		self.bot_acceleration = [0.0, 0.0, 0.0]

		self.bot_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
		self.bot_orientation_euler = [0.0, 0.0, 0.0]
		self.bot_angular_velocity = [0.0, 0.0, 0.0]

		# height of com
		self.l = 0.5*(2**0.5)
		
		#                         x_vel, pitch_vel, yaw_vel, x, pitch, yaw
		self.set_point = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])


		self.Q = np.array([[1, 0, 0, 0, 0, 0], 
						   [0, 10, 0, 0, 0, 0], 
						   [0, 0, 0.1, 0, 0, 0], 
						   [0, 0, 0, 100, 0, 0], 
						   [0, 0, 0, 0, 1000, 0], 
						   [0, 0, 0, 0, 0, 1]])

		self.R = np.array([[1, 0],
						   [0, 1]])

		

		# make the publisher to publish the motor effort value###################################
		self.left_wheel_vel = rospy.Publisher('/teeterbot/left_wheel_controller/command', Float64, queue_size=10)
		self.right_wheel_vel = rospy.Publisher('/teeterbot/right_wheel_controller/command', Float64, queue_size=10)
		
		self.height_pub = rospy.Publisher('/height', Float64, queue_size=10)
		
		self.pitch_up = rospy.Publisher('/pitch_up', Float32, queue_size=1)
		self.pitch_low = rospy.Publisher('/pitch_low', Float32, queue_size=1)
		self.pitch_error = rospy.Publisher('/pitch_error', Float32, queue_size=1)

		# update the subscriber to get the sensor data##########################################
		rospy.Subscriber('/imu', Imu, self.imu_callback)
		rospy.Subscriber('/gps', NavSatFix, self.gps_callback)
		rospy.Subscriber('/gps_velocity', Vector3Stamped, self.gps_vel_callback)
		rospy.Subscriber("/h_com", Float64 , self.h_com_value)
	

	def h_com_value(self, data):
		self.l = data.data;
		# print("val recived ", self.l)


	def imu_callback(self, data):
		self.bot_orientation_quaternion[0] = data.orientation.x
		self.bot_orientation_quaternion[1] = data.orientation.y
		self.bot_orientation_quaternion[2] = data.orientation.z
		self.bot_orientation_quaternion[3] = data.orientation.w

		(self.bot_orientation_euler[0], self.bot_orientation_euler[1], self.bot_orientation_euler[2]) = tf.transformations.euler_from_quaternion(
            [self.bot_orientation_quaternion[0], self.bot_orientation_quaternion[1], self.bot_orientation_quaternion[2], self.bot_orientation_quaternion[3]])
		# print("ok")
		self.bot_angular_velocity[0] = data.angular_velocity.x
		self.bot_angular_velocity[1] = data.angular_velocity.y
		self.bot_angular_velocity[2] = data.angular_velocity.z

		self.bot_acceleration[0] = -data.linear_acceleration.x
		self.bot_acceleration[1] = -data.linear_acceleration.y
		self.bot_acceleration[2] = -data.linear_acceleration.z


	def gps_callback(self, data):
		x = (data.latitude - 49.9)/0.0000091149
		y = -(data.longitude - 8.9)/0.0000138728
		z = data.altitude

		# frame transformation
		self.bot_location[0] = x*np.cos(self.bot_orientation_euler[2]) + y*np.sin(self.bot_orientation_euler[2])
		self.bot_location[1] = x*np.sin(self.bot_orientation_euler[2]) - y*np.cos(self.bot_orientation_euler[2])
		self.bot_location[2] = z
		# print("location ", self.bot_location)

	def gps_vel_callback(self, data):
		x = data.vector.x
		y = data.vector.y
		z = data.vector.z

		self.bot_velocity[0] = x*np.cos(self.bot_orientation_euler[2]) + y*np.sin(self.bot_orientation_euler[2])
		self.bot_velocity[1] = x*np.sin(self.bot_orientation_euler[2]) - y*np.cos(self.bot_orientation_euler[2])
		self.bot_velocity[2] = z
		# print("bot_velocity ", self.bot_velocity)


	def LQR(self):
		l = self.l
		self.A = np.array([[-(520000*l**2 + 104000*l + 20835)/(2*(1560000*l**2 + 279189)), (104000*l**2 + 20800*l + 4167)/(2*(1560000*l**2 + 279189)),                                 0, 0, -(53052480*l**2)/(1560000*l**2 + 279189), 0], 
						   [(40*(6500*l + 1675))/(1560000*l**2 + 279189),              -(40*(1300*l + 335))/(1560000*l**2 + 279189),                                 0, 0,    (68356080*l)/(1560000*l**2 + 279189), 0], 
						   [0,                                                        0, -613214628862394/7627737681678155, 0,                                      0, 0], 
						   [1, 0, 0, 0, 0, 0], 
						   [0, 1, 0, 0, 0, 0], 
						   [0, 0, 1, 0, 0, 0]])
		
		self.B = np.array([[(5200000*l*l + 1040000*l + 208350)/(2*(1560000*l*l + 279189)), (5200000*l*l + 1040000*l + 208350)/(2*(1560000*l*l + 279189)) ], 
						   [-(40*(65000*l + 16750))/(1560000*l*l + 279189), -(40*(65000*l + 16750))/(1560000*l*l + 279189)], 
						   [-2.178664673, 2.178664673], 
						   [0.0 , 0.0], 
						   [0, 0      ], 
						   [0.0, 0.0 ]])

		K, S, e = lqr(self.A, self.B, self.Q, self.R)

		error_matrix = np.array([[self.set_point[0, 0] - self.bot_velocity[0]],
								 [self.set_point[1, 0] - self.bot_angular_velocity[1]],
								 [self.set_point[2, 0] - self.bot_angular_velocity[2]],
								 [self.set_point[3, 0] - self.bot_location[0]],
								 [self.set_point[4, 0] - self.bot_orientation_euler[1]],
								 [self.set_point[5, 0] - self.bot_orientation_euler[2]]])

		output = np.dot(K, error_matrix)


		self.left_wheel_vel.publish(output[0])
		self.right_wheel_vel.publish(output[1])

		# self.pitch_up.publish(0)
		# self.pitch_low.publish(-5)
		# self.pitch_error.publish(self.error_matrix[2, 0]*180/np.pi)


def main():
	print("main")
	t = time.time()	
	# stablizing
	while time.time()-t <10:
		bot.LQR()
		time.sleep(0.01)

	bot.set_point[0] = 0.0
	bot.set_point[2] = 0.0
	bot.set_point[3] = bot.bot_location[0]
	bot.set_point[5] = bot.bot_orientation_euler[2]
	# # siting to 0.5
	h = 0.5*(2**0.5)
	while h>0.5:
		bot.height_pub.publish(h)
		h -= 0.005
		bot.LQR()
		time.sleep(0.01)
	
	# t = time.time()
	# while time.time()-t <2:
		
	# 	bot.LQR()
	# 	time.sleep(0.01)

	while h<0.8:
		bot.height_pub.publish(h)
		h += 0.025
		bot.LQR()
		time.sleep(0.01)
	
	# t = time.time()
	# while time.time()-t <2:
		
	# 	bot.LQR()
	# 	time.sleep(0.01)

	while h>0.5:
		bot.height_pub.publish(h)
		h -= 0.02
		bot.LQR()
		time.sleep(0.01)

	
	while True:
		t = time.time()
		while(time.time()-t<20):
			bot.set_point[0] = 0.0
			bot.set_point[2] = 0.0
			bot.set_point[3] = 0 #bot.bot_location[0]
			bot.set_point[5] = 0 #bot.bot_orientation_euler[2]
			# print(time.time()-t)
			bot.LQR()
			time.sleep(0.01)

		t = time.time()
		while(time.time()-t<20):
			bot.set_point[0] = 0.0
			bot.set_point[2] = 0.0
			bot.set_point[3] = 0 #bot.bot_location[0]
			bot.set_point[5] = 0 #bot.bot_orientation_euler[2]

			bot.LQR()
			time.sleep(0.01)


if __name__ == '__main__':

    # making e_drone object of Edrone class
    bot = lwr_bot()

    # pause of 1 sec
    t = time.time()
    while time.time() - t < 1:
        pass

    while not rospy.is_shutdown():
        main()
        break
