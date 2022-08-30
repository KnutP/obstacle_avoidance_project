#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from math import pi, atan2

class Controller:
	def __init__(self, P=0.0, D=0.0, set_point=0):
		self.Kp = P
		self.Kd = D
		self.set_point = set_point # reference (desired value of theta)
		self.previous_error = 0

	def update(self, current_value):
		# current_value is theta
		# calculate P_term and D_term
		error = self.set_point - current_value
		P_term = self.Kp*error 
		D_term = self.Kd*(error-self.previous_error)
		self.previous_error = error
		return P_term + D_term
	
	def setPoint(self, set_point):
		self.set_point = set_point
		self.previous_error = 0
	
	def setPD(self, P, D):
		self.Kp = P
		self.Kd = D

class Turtlebot():
	def __init__(self):
		rospy.init_node("turtlebot_move")
		rospy.loginfo("Press Ctrl + C to terminate")
		self.control = Controller()
		self.vel = Twist()
		self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
		self.rate = rospy.Rate(10)

		# reset odometry to zero
		# if subscribe to odom is before this, robot will not move if this section is commented
		self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
		for i in range(10):
			self.reset_pub.publish(Empty())
			self.rate.sleep()

		# subscribe to odometry
		self.pose = Pose2D()
		self.logging_counter = 0
		self.trajectory = list()
		rospy.loginfo("created traj")
		self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

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
		seg2stat = 0
		seg3stat = 0
		seg4stat = 0
		velstart = 0
		ct = 0
		while not rospy.is_shutdown():
			# segment 1 (4,0)
			while seg1stat == 0:
				x1 = 4
				y1 = 0
				desdir = atan2((y1 - self.pose.y),(x1 - self.pose.x))
				Controller.setPoint(self.control,desdir)
				Controller.setPD(self.control,2.0,2.0)
				while velstart == 0:
					rec = Controller.update(self.control, self.pose.theta)
					rospy.loginfo("Rotating before start:" + str(rec))
					self.vel.angular.z = rec
					self.vel.linear.x = 0
					self.vel_pub.publish(self.vel)
					if ((self.pose.theta < (desdir+0.04)) and (self.pose.theta > (desdir-0.04))):
						velstart = 1
				if [self.pose.x, self.pose.y] != [x1,y1]:
					rec = Controller.update(self.control, self.pose.theta)
					rospy.loginfo("Recommended from PD:" + str(rec))
					self.vel.angular.z = rec
				if (self.pose.y > (y1-0.15) and self.pose.y < (y1+0.15)):
					if (self.pose.x < (x1-0.05)):
						self.vel.linear.x = 1.0
					else:
						self.vel.linear.x = 0
						seg1stat = 1
				else:
					self.vel.linear.x = 0
					seg1stat = 1
				self.vel_pub.publish(self.vel)
				rospy.loginfo(self.vel)
				rospy.loginfo("running")
				self.rate.sleep()
			# segment 2 (4,4)
			if (seg1stat == 1 and velstart == 1):
				x2 = 4
				y2 = 4
				desdir = atan2((y2 - self.pose.y),(x2 - self.pose.x))
				Controller.setPoint(self.control,desdir)
				Controller.setPD(self.control,2.5,2.5)
			while (velstart == 1 and seg1stat == 1):
				if ct == 550:
					break
				ct += 1
				rec = Controller.update(self.control, self.pose.theta)
				rospy.loginfo("Rotating STEP2 start:" + str(rec))
				rospy.loginfo("Current theta" + str(self.pose.theta) + "; Desired theta" + str(desdir))
				self.vel.angular.z = rec
				rospy.loginfo(self.vel.angular.z)
				self.vel.linear.x = 0
				self.vel_pub.publish(self.vel)
				if ((self.pose.theta < (desdir+0.04)) and (self.pose.theta > (desdir-0.04))):
					velstart = 2
					break
				self.rate.sleep()
			while (seg2stat == 0 and seg1stat == 1 and velstart == 2):
				x2 = 4
				y2 = 4
				ct = 0
				desdir = atan2((y2 - self.pose.y),(x2 - self.pose.x))
				Controller.setPoint(self.control,desdir)
				Controller.setPD(self.control,2.0,2.0)
				if [self.pose.x, self.pose.y] != [x2,y2]:
					rec = Controller.update(self.control, self.pose.theta)
					rospy.loginfo("Recommended from PD:" + str(rec))
					self.vel.angular.z = rec
  				# recall  that the velocity needs to be constant, so these if statements gotta go
				if (self.pose.x > (x2-0.15) and self.pose.x < (x2+0.15)):
					if (self.pose.y < (y2-0.08)):
						self.vel.linear.x = 1.0
					else:
						self.vel.linear.x = 0
						seg2stat = 1
				else:
					self.vel.linear.x = 0
					seg2stat = 1
				self.vel_pub.publish(self.vel)
				rospy.loginfo(self.vel)
				rospy.loginfo("running")
				self.rate.sleep()
 			# segment 3 (0,4)
			if (seg2stat == 1 and velstart == 2):
				x3 = 0
				y3 = 4
				desdir = atan2((y3 - self.pose.y),(x3 - self.pose.x))
				if desdir < 0:
					desdir = pi + (pi - (desdir*-1))
				else: 
					desdir = desdir
				Controller.setPoint(self.control,desdir)
				Controller.setPD(self.control,2.5,2.5)
			while (velstart == 2 and seg2stat == 1):
				if ct == 550:
					break
				ct += 1
				if self.pose.theta < 0:
					thetainput = pi + (pi - (self.pose.theta * -1))
				else:
					thetainput = self.pose.theta
				rec = Controller.update(self.control, thetainput)
				rospy.loginfo("Rotating STEP3 start:" + str(rec))
				rospy.loginfo("Current theta" + str(self.pose.theta) + "; Input theta:" + str(thetainput) + "; Desired theta" + str(desdir))
				self.vel.angular.z = rec
				rospy.loginfo(self.vel.angular.z)
				self.vel.linear.x = 0
				self.vel_pub.publish(self.vel)
				if ((thetainput < (desdir+0.04)) and (thetainput > (desdir-0.04))):
					velstart = 3
					break
				self.rate.sleep()
			while (seg3stat == 0 and seg2stat == 1 and velstart == 3):
				x3 = 0
				y3 = 4
				ct = 0
				desdir = atan2((y3 - self.pose.y),(x3 - self.pose.x))
				if desdir < 0:
					desdir = pi + (pi - (desdir*-1))
				else: 
					desdir = desdir
				Controller.setPoint(self.control,desdir)
				Controller.setPD(self.control,2.0,2.0)
				if [self.pose.x, self.pose.y] != [x3,y3]:
					if self.pose.theta < 0:
						thetainput = pi + (pi - (self.pose.theta * -1))
					else:
						thetainput = self.pose.theta
					rospy.loginfo("Theta input:" + str(thetainput))
					rec = Controller.update(self.control, thetainput)
					rospy.loginfo("Recommended from PD:" + str(rec))
					self.vel.angular.z = rec
				if (self.pose.y > (y3-0.15) and self.pose.y < (y3+0.15)):
					if (self.pose.x > (x3 + 0.08)):
						self.vel.linear.x = 1.0
					else:
						self.vel.linear.x = 0
						seg3stat = 1
				else:
					self.vel.linear.x = 0
					seg3stat = 1
				self.vel_pub.publish(self.vel)
				rospy.loginfo(self.vel)
				rospy.loginfo("running")
				self.rate.sleep()
  			# segment 4 (0,4)
			if (seg3stat == 1 and velstart == 3):
				x4 = 0
				y4 = 0
				desdir = atan2((y4 - self.pose.y),(x4 - self.pose.x))
				if desdir < 0:
					desdir = pi + (pi - (desdir*-1))
				else: 
					desdir = desdir
				Controller.setPoint(self.control,desdir)
				Controller.setPD(self.control,2.5,2.5)
			while (velstart == 3 and seg3stat == 1):
				if ct == 550:
					break
				ct += 1
				if self.pose.theta < 0:
					thetainput = pi + (pi - (self.pose.theta * -1))
				else:
					thetainput = self.pose.theta
				rec = Controller.update(self.control, thetainput)
				rospy.loginfo("Rotating STEP4 start:" + str(rec))
				rospy.loginfo("Current theta" + str(self.pose.theta) + "; Input theta:" + str(thetainput) + "; Desired theta" + str(desdir))
				self.vel.angular.z = rec
				rospy.loginfo(self.vel.angular.z)
				self.vel.linear.x = 0
				self.vel_pub.publish(self.vel)
				if ((thetainput < (desdir+0.04)) and (thetainput > (desdir-0.04))):
					velstart = 4
					break
				self.rate.sleep()
			while (seg4stat == 0 and seg3stat == 1 and velstart == 4):
				x4 = 0
				y4 = 0
				ct = 0
				desdir = atan2((y4 - self.pose.y),(x4 - self.pose.x))
				if desdir < 0:
					desdir = pi + (pi - (desdir*-1))
				else: 
					desdir = desdir
				Controller.setPoint(self.control,desdir)
				Controller.setPD(self.control,2.0,2.0)
				if [self.pose.x, self.pose.y] != [x4,y4]:
					if self.pose.theta < 0:
						thetainput = pi + (pi - (self.pose.theta * -1))
					else:
						thetainput = self.pose.theta
					rospy.loginfo("Theta input:" + str(thetainput))
					rec = Controller.update(self.control, thetainput)
					rospy.loginfo("Recommended from PD:" + str(rec))
					self.vel.angular.z = rec
				if (self.pose.x > (x4-0.15) and self.pose.x < (x4+0.15)):
					if (self.pose.y > (y4 + 0.1)):
						self.vel.linear.x = 1.0
					else:
						self.vel.linear.x = 0
						seg4stat = 1
				else:
					self.vel.linear.x = 0
					seg4stat = 1
				self.vel_pub.publish(self.vel)
				rospy.loginfo(self.vel)
				rospy.loginfo("running")
				self.rate.sleep()
			rospy.loginfo("finished")
			break
		# remove the code above and add your code here to adjust your movement based on 2D pose feedback

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