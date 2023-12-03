#!/usr/bin/env python

# joint controller to lift abhimanyu at certain height using LQR controller 
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
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3Stamped

# robot description
d = 0.3      #0.29;
l = 0.4      #0.2907;
r = 0.2      #0.07;
mb = 10      #4;
mw = 1      #0.1;
J = 0.02      #0.000735;
K = 0.01833      #0.00039;
I1 = 0.15      #0.015625;
I2 = 0.6083      #0.0118625;
I3 = 0.6083      #0.0118625;
calpha = 0.0;
g = 9.81;


class lwr_bot():
	def __init__(self):
		rospy.init_node('lwr_joint_pd')

		self.bot_location = [0.0, 0.0, 0.0]
		self.bot_velocity = [0.0, 0.0, 0.0]
		self.bot_acceleration = [0.0, 0.0, 0.0]

		self.bot_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
		self.bot_orientation_euler = [0.0, 0.0, 0.0]
		self.bot_angular_velocity = [0.0, 0.0, 0.0]

		# height of com
		self.l = 0.0
		
		# joint motors anlge theta1, theta2(knee), theta3(hip)
		self.theta_l = [0.0, 0.0, 0.0]
		self.theta_r = [0.0, 0.0, 0.0]

		self.joint_pos_l = [np.pi/4, np.pi/2, -np.pi/4]
		self.joint_vel_l = [0, 0, 0]

		self.joint_pos_r = [np.pi/4, np.pi/2, -np.pi/4]
		self.joint_vel_r = [0, 0, 0]

		self.joint_pos_setpoint_l = [np.pi/4, np.pi/2, -np.pi/4]
		self.joint_vel_setpoint_l = [0, 0, 0]

		self.joint_pos_setpoint_r = [np.pi/4, np.pi/2, -np.pi/4]
		self.joint_vel_setpoint_r = [0, 0, 0]

		# height of legs 
		self.h_l = 0.5*(2**0.5)
		self.h_l = 0.5*(2**0.5)
		self.h_l_vel = 0.0
		self.h_r_vel = 0.0

		# given height h of robot 
		self.height = 0.5*(2**0.5)
		# differnce the heights of both the legs
		self.delta_h = 0.0
		# note that h_l = height + delta_h/2   and   h_r = height - delta_h/2

		# setpoint roll angle 
		self.set_point_roll =0.0
		# tune kp_roll properly for roll disturbance rejection
		self.kp_roll = 0.0

		self.Q = np.array([[1, 0, 0, 0], 
						   [0, 0.1, 0, 0], 
						   [0, 0, 1, 0], 
						   [0, 0, 0, 0.1]])

		self.R = np.array([[100000, 0],
						   [0, 100000]])


		# make the publisher to publish the joint motor angle value ###################################
		self.theta2_l_pub = rospy.Publisher('/teeterbot/left_calf_controller/command', Float64, queue_size=10)
		self.theta3_l_pub = rospy.Publisher('/teeterbot/left_thigh_controller/command', Float64, queue_size=10)
		self.theta2_r_pub = rospy.Publisher('/teeterbot/right_calf_controller/command', Float64, queue_size=10)
		self.theta3_r_pub = rospy.Publisher('/teeterbot/right_thigh_controller/command', Float64, queue_size=10)
		

		# publish h_com for lwr_wheel_lqr controller
		self.h_com_pub = rospy.Publisher('/h_com', Float64, queue_size=10)

		# update the subscriber to get the sensor data ##########################################
		rospy.Subscriber('/imu', Imu, self.imu_callback)
		rospy.Subscriber('/gps', NavSatFix, self.gps_callback)
		rospy.Subscriber('/gps_velocity', Vector3Stamped, self.gps_vel_callback)
		rospy.Subscriber("/height", Float64 , self.get_height)
		rospy.Subscriber("/teeterbot/joint_states", JointState , self.get_jointstate)

	def get_jointstate(self, data):
		self.joint_pos_l[1] = data.position[0]
		self.joint_pos_l[2] = data.position[1]

		self.joint_pos_r[1] = data.position[3]
		self.joint_pos_r[2] = data.position[4]

		self.joint_vel_l[1] = data.velocity[0]
		self.joint_vel_l[2] = data.velocity[1]

		self.joint_vel_r[1] = data.velocity[3]
		self.joint_vel_r[2] = data.velocity[4]

	
	def get_height(self, data):
		self.height = data.data;

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

		# frame transformation global to robot 
		self.bot_location[0] = x*np.cos(self.bot_orientation_euler[2]) + y*np.sin(self.bot_orientation_euler[2])
		self.bot_location[1] = x*np.sin(self.bot_orientation_euler[2]) - y*np.cos(self.bot_orientation_euler[2])
		self.bot_location[2] = z
		# print("location ", self.bot_location)

	def gps_vel_callback(self, data):
		x = data.vector.x
		y = data.vector.y
		z = data.vector.z

		# frame transformation global to robot 
		self.bot_velocity[0] = x*np.cos(self.bot_orientation_euler[2]) + y*np.sin(self.bot_orientation_euler[2])
		self.bot_velocity[1] = x*np.sin(self.bot_orientation_euler[2]) - y*np.cos(self.bot_orientation_euler[2])
		self.bot_velocity[2] = z
		# print("bot_velocity ", self.bot_velocity)


	def PID(self):
		
		self.set_point_roll = np.arctan(-self.bot_velocity[0]*self.bot_angular_velocity[2]/g)
		# self.delta_h = np.tan(self.set_point_roll-self.bot_orientation_euler[0])*d
		# self.delta_h = -self.bot_velocity[0]*self.bot_angular_velocity[2]*d/g

		# restriction roll anngle to -20 to 20 degree only 
		if(self.set_point_roll < -20*np.pi/180):
			self.set_point_roll = -20*np.pi/180
		elif(self.set_point_roll > 20*np.pi/180):
			self.set_point_roll = 20*np.pi/180

		self.delta_h = np.tan(self.set_point_roll)*d + self.kp_roll*np.tan(self.set_point_roll-self.bot_orientation_euler[0])*d
		# print(self.delta_h)
		self.h_l = self.height + self.delta_h/2
		self.h_r = self.height - self.delta_h/2

		# put a height restriction also ###############################################
		# core here



		# equation to have theta 2 and 3 of (l and r)
		self.theta_l[0] = np.arcsin(self.h_l/(2*0.5))
		self.theta_l[1] = np.pi - 2*self.theta_l[0]
		self.theta_l[2] = np.pi/2 - self.theta_l[0] - self.theta_l[1]
		
		self.theta_r[0] = np.arcsin(self.h_r/(2*0.5))
		self.theta_r[1] = np.pi - 2*self.theta_r[0]
		self.theta_r[2] = np.pi/2 - self.theta_r[0] - self.theta_r[1]

		self.joint_pos_setpoint_l[0] = self.theta_l[0]
		self.joint_pos_setpoint_l[1] = self.theta_l[1]
		self.joint_pos_setpoint_l[2] = self.theta_l[2]
		self.joint_pos_setpoint_r[0] = self.theta_r[0]
		self.joint_pos_setpoint_r[1] = self.theta_r[1]
		self.joint_pos_setpoint_r[2] = self.theta_r[2]

		self.joint_vel_setpoint_l[0] = self.h_l_vel/(2*0.5*np.cos(self.joint_pos_setpoint_l[0]))
		self.joint_vel_setpoint_l[1] = -2*self.joint_vel_setpoint_l[0]
		self.joint_vel_setpoint_l[2] = -self.joint_vel_setpoint_l[0] - self.joint_vel_setpoint_l[1]
		self.joint_vel_setpoint_r[0] = self.h_r_vel/(2*0.5*np.cos(self.joint_pos_setpoint_r[0]))
		self.joint_vel_setpoint_r[1] = -2*self.joint_vel_setpoint_r[0]
		self.joint_vel_setpoint_r[2] = -self.joint_vel_setpoint_r[0] - self.joint_vel_setpoint_r[1]

		k_l = 8.34*np.cos(self.joint_pos_setpoint_l[1]/2)
		k_r = 8.34*np.cos(self.joint_pos_setpoint_r[1]/2)
		p = 0.3428
		q = 2.34
		A_l = np.array([[0, 1, 0, 0],
					 [k_l, 0, 0, 0],
					 [0, 0, 0, 1],
					 [0, 0, 0, 0]])
		A_r = np.array([[0, 1, 0, 0],
					 [k_r, 0, 0, 0],
					 [0, 0, 0, 1],
					 [0, 0, 0, 0]])
		B = np.array([[0, 0],
					 [p, 0],
					 [0, 0],
					 [0, q]])

		K_l, S, e = lqr(A_l, B, self.Q, self.R)
		K_r, S, e = lqr(A_r, B, self.Q, self.R)

		error_matrix_l = np.array([[self.joint_pos_setpoint_l[1]-self.joint_pos_l[1]],
								 [self.joint_vel_setpoint_l[1]-self.joint_vel_l[1]],
								 [self.joint_pos_setpoint_l[2]-self.joint_pos_l[2]],
								 [self.joint_vel_setpoint_l[2]-self.joint_vel_l[2]]])

		output_l = np.dot(K_l, error_matrix_l)

		error_matrix_r = np.array([[self.joint_pos_setpoint_r[1]-self.joint_pos_r[1]],
								 [self.joint_vel_setpoint_r[1]-self.joint_vel_r[1]],
								 [self.joint_pos_setpoint_r[2]-self.joint_pos_r[2]],
								 [self.joint_vel_setpoint_r[2]-self.joint_vel_r[2]]])

		output_r = np.dot(K_r, error_matrix_r)
		# print(output_r)

		# passing the value with a -ve to make the lages fold in backward pose as shown in fig above
		self.theta2_l_pub.publish(output_l[0])
		self.theta3_l_pub.publish(output_l[1])
		self.theta2_r_pub.publish(output_r[0])
		self.theta3_r_pub.publish(output_r[1])
		# self.theta2_l_pub.publish(-self.theta_l[1])
		# self.theta3_l_pub.publish(-self.theta_l[2])
		# self.theta2_r_pub.publish(-self.theta_r[1])
		# self.theta3_r_pub.publish(-self.theta_r[2])


		# calculate h_com based on theta
		self.l = (self.h_l+self.h_r)/2 ########################################################################
		self.h_com_pub.publish(self.l)


def main():
	print("main")
	t = time.time()
	while True:
		bot.PID()
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
