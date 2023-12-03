#!/usr/bin/env python

import rospy
import tf
import numpy as np
import time
from std_msgs.msg import Float64, Float32
from sensor_msgs.msg import Imu
from pid_tune.msg import PidTune

class sb_bot():
	def __init__(self):
		rospy.init_node('sb_PID')
		self.bot_location = [0.0, 0.0, 0.0]
		self.bot_velocity = [0.0, 0.0, 0.0]

		self.bot_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
		self.bot_orientation_euler = [0.0, 0.0, 0.0]
		self.bot_angular_velocity = [0.0, 0.0, 0.0]

		self.set_angle = 0.0

		self.error = 0
		self.diff_error = 0
		self.prev_error = 0
		self.sum_error = 0


		self.Kpid = [25, 1, 500]

		self.left_wheel_vel = rospy.Publisher('/teeterbot/left_wheel_controller/command', Float64, queue_size=10)
		self.right_wheel_vel = rospy.Publisher('/teeterbot/right_wheel_controller/command', Float64, queue_size=10)
		
		self.pitch_up = rospy.Publisher('/pitch_up', Float32, queue_size=1)
		self.pitch_low = rospy.Publisher('/pitch_low', Float32, queue_size=1)
		self.pitch_error = rospy.Publisher('/pitch_error', Float32, queue_size=1)

		rospy.Subscriber('/teeterbot/imu', Imu, self.imu_callback)
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
		rospy.Subscriber("/set_angle", Float64 , self.set_angle_value)
	
	def set_angle_value(self, data):
		self.set_angle = data.data;


	def pitch_set_pid(self, pitch):
		self.Kpid[0] = pitch.Kp
		self.Kpid[1] = pitch.Ki * 0.1
		self.Kpid[2] = pitch.Kd * 0.1
		print(self.Kpid)

	def imu_callback(self, data):
		self.bot_orientation_quaternion[0] = data.orientation.x
		self.bot_orientation_quaternion[1] = data.orientation.y
		self.bot_orientation_quaternion[2] = data.orientation.z
		self.bot_orientation_quaternion[3] = data.orientation.w

		(self.bot_orientation_euler[0], self.bot_orientation_euler[1], self.bot_orientation_euler[2]) = tf.transformations.euler_from_quaternion(
            [self.bot_orientation_quaternion[0], self.bot_orientation_quaternion[1], self.bot_orientation_quaternion[2], self.bot_orientation_quaternion[3]])
		# print(self.bot_orientation_euler)


	def pid(self):
		
		self.error = self.set_angle*np.pi/180 - self.bot_orientation_euler[1]
		self.sum_error += self.error
		self.diff_error = self.error - self.prev_error
		self.prev_error = self.error

		output = self.Kpid[0]*self.error + self.Kpid[1]*self.sum_error + self.Kpid[2]*self.diff_error

		self.left_wheel_vel.publish(-output)
		self.right_wheel_vel.publish(-output)

		self.pitch_up.publish(0)
		self.pitch_low.publish(-5)
		self.pitch_error.publish(self.error*180/np.pi)


def main():
	print("main")
	t = time.time()
	while True:
		bot.pid()
		time.sleep(0.01)


if __name__ == '__main__':

    # making e_drone object of Edrone class
    bot = sb_bot()

    # pause of 1 sec
    t = time.time()
    while time.time() - t < 1:
        pass

    while not rospy.is_shutdown():
        main()
        break
