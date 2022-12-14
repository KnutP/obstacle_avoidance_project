#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import pi, atan2, cos
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

class Controller:
	def __init__(self, P=0.0, I=0.0, D=0.0, set_point=0, Time_Step = 0.005):
		self.Time_Step = Time_Step
		self.Kp = P
		self.Ki = I
		self.Kd = D
		self.set_point = set_point  # reference (desired value of theta)
		self.error = 0
		self.previous_error = 0
		self.integral_error = 0
		self.derivative_error = 0
		self.output = 0

	def update(self, current_value):
        # current_value is theta
        # calculate P_term and D_term
		self.error = self.set_point - current_value
		derivative_error = (self.error - self.previous_error) / self.Time_Step
		self.integral_error += self.error * self.Time_Step
		P_term = self.Kp*self.error
		I_term = self.Ki*self.integral_error
		D_term = self.Kd*derivative_error
		output = P_term +I_term + D_term
		self.previous_error = self.error
		return output
	
	def setPoint(self, set_point):
		self.set_point = set_point
		self.previous_error = 0

	def setPID(self, P, I, D):
		self.Kp = P
		self.Ki = I
		self.Kd = D
	
	def setPoint(self, set_point):
		self.set_point = set_point
		self.previous_error = 0

	def setPID(self, P, I, D):
		self.Kp = P
		self.Ki = I
		self.Kd = D

class Turtlebot():
	def __init__(self):
		rospy.init_node("turtlebot_move")
		rospy.loginfo("Press Ctrl + C to terminate")
		self.control = Controller()
		self.vel = Twist()
		# new line above for obstacle override velocity
		self.vel_obs = Twist()
		self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.rate = rospy.Rate(10)
		self.isObstacle = False
		self.direction = 1

		# reset odometry to zero
		# if subscribe to odom is before this, robot will not move if this section is commented
		self.reset_pub = rospy.Publisher("reset/std_msgs", Empty, queue_size=10)
		# you can also replace the quotes with mobile_base/commands/reset_odometry
		for i in range(10):
			self.reset_pub.publish(Empty())
			self.rate.sleep()

		# subscribe to odometry
		self.pose = Pose2D()
		self.logging_counter = 0
		self.trajectory = list()
		rospy.loginfo("created traj")
		self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

		self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback)  # Subscriber object which will listen "LaserScan" type messages
                                                      # from the "/scan" Topic and call the "callback" function
						      # each time it reads something from the Topic

		try:
			self.run()
		except rospy.ROSInterruptException:
			rospy.loginfo("Action terminated.")
		finally:
			# save trajectory into csv file
			rospy.loginfo("Sending csv")
			np.savetxt('trajectory.csv', np.array(self.trajectory), delimiter=',', fmt='%f')

	def run(self):
		seg1stat = 0
		velstart = 0
		while not rospy.is_shutdown():
			# segment 1 (4,0)
			while seg1stat == 0:
				x1 = 4
				y1 = 0
				desdir = atan2((y1 - self.pose.y),(x1 - self.pose.x))

				Controller.setPoint(self.control,desdir)
				Controller.setPID(self.control, 1.2 , 0.03, 0.005)

				while velstart == 0:
					rec = Controller.update(self.control, self.pose.theta)
					rospy.loginfo("Rotating before start:" + str(rec))
					self.vel.angular.z = rec
					# we assume no obstacle direclty in front of robot starting point
					self.vel.linear.x = 0
					self.vel_pub.publish(self.vel)
					if ((self.pose.theta < (desdir+0.04)) and (self.pose.theta > (desdir-0.04))):
						velstart = 1

				if [self.pose.x, self.pose.y] != [x1,y1]:
					rec = Controller.update(self.control, self.pose.theta)
					rospy.loginfo("Recommended from PID:" + str(rec))
					if self.vel_obs.angular.z > 0 or self.isObstacle:
						self.vel.angular.z = self.vel_obs.angular.z
						self.vel.linear.x = self.vel_obs.linear.x
					else:
						self.vel.angular.z = rec
						self.vel.linear.x = self.vel_obs.linear.x

				# Check if we're at the target
				if (self.pose.y > (y1-5) and self.pose.y < (y1+5)):
					if (self.pose.x < (x1-0.05)):
						self.vel.linear.x = self.vel_obs.linear.x
					else:
						self.vel.linear.x = 0
						self.vel.angular.z = 0
						seg1stat = 1
				else:
					self.vel.linear.x = 0
					self.vel.angular.z = 0
					seg1stat = 1
				self.vel_pub.publish(self.vel)
				rospy.loginfo(self.vel)
				rospy.loginfo("running")
				self.rate.sleep()
			
				#rospy.loginfo(self.vel)
				#rospy.loginfo("running")
				#self.rate.sleep()
			rospy.loginfo("finished")
			break
		# remove the code above and add your code here to adjust your movement based on 2D pose feedback

	def callback(self, dt):
		print ('-------------------------------------------')
		print (dt.ranges[0])
		print (dt.ranges[15])
		print (dt.ranges[345])
		print ('-------------------------------------------')
		thr1 = 0.32 # Laser scan range threshold
		thr2 = 0.22

		if not self.isObstacle:
			# if obstacle in front and open to the right, turn right
			if (dt.ranges[0] < (thr1+0.1) and (dt.ranges[20] > 2*thr1 or dt.ranges[45] > 2*thr1)):
				self.direction = 1
			# if obstacle in front and open to the left, turn left
			elif (dt.ranges[0] < (thr1+0.1) and (dt.ranges[340] > 2*thr1 or dt.ranges[315] > 2*thr1)):
				self.direction = -1
			# if obstacle slopes to the right, turn left
			elif (dt.ranges[5] < dt.ranges[0]/cos(5*180/pi) or dt.ranges[15] < dt.ranges[0]/cos(15*180/pi)) and (dt.ranges[0] < (thr1+0.1)
					or dt.ranges[5] < (thr1+0.1) or dt.ranges[15] < (thr1+0.1)):
				self.direction = -1
			# if obstacle slopes to the left, turn right
			elif (dt.ranges[355] < dt.ranges[0]/cos(5*180/pi) or dt.ranges[345] < dt.ranges[0]/cos(15*180/pi)) and (dt.ranges[0] < (thr1+0.1)
					or dt.ranges[355] < (thr1+0.1) or dt.ranges[345] < (thr1+0.1)):
				self.direction = 1
		
		# if no obstacle, drive forward
		if dt.ranges[0]>thr1 and dt.ranges[15]>thr1 and dt.ranges[30]>thr2 and dt.ranges[345]>thr1 and dt.ranges[330]>thr2:
			self.vel_obs.linear.x = 0.2
			self.vel_obs.angular.z = 0.0
			self.isObstacle = False
		# if there is an obstacle, turn away from it
		else:
			self.vel_obs.linear.x = 0.0 # stop
			self.vel_obs.angular.z = 0.5*self.direction # rotate based on direction chosen earlier
			self.isObstacle = True

		# if there is still an obstacle at the sides, override turning and keep going straight
		if dt.ranges[90]<thr2 or dt.ranges[45]<thr1 or dt.ranges[270]<thr2  or dt.ranges[315]<thr1:
			self.isObstacle = True


	def odom_callback(self, msg):
		# get pose = (x, y, theta) from odometry topic
		quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion (quarternion)
		self.pose.theta = yaw
		self.pose.x = msg.pose.pose.position.x
		self.pose.y = msg.pose.pose.position.y
		
		# logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
		self.logging_counter += 1
		if self.logging_counter == 100:
			rospy.loginfo("got to if")
			self.logging_counter = 0
			self.trajectory.append([self.pose.x, self.pose.y]) # save trajectory
			rospy.loginfo("odom: x=" + str(self.pose.x) + "; y=" + str(self.pose.y) + "; theta=" + str(yaw))
			rospy.loginfo(np.array(self.trajectory))
if __name__ == '__main__':
	whatever = Turtlebot()