#!/usr/bin/env python3
from readchar import readchar as rc
from asyncio import run as r

from math import pi

import sys
import errno
import socket
import threading
import rclpy
import select
import time
from miscellaneous import constrain_angle, remove_spaces
import numpy as np
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from control_msgs.action import FollowJointTrajectory
from launch.substitutions import LaunchConfiguration
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu, PointCloud2
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer
from gazebo_msgs.srv import GetModelState, GetEntityState
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from random import randrange as rr
from teleop_twist_keyboard import getKey as gk

class LegController(Node):
	def __init__(self):
		super().__init__('leg_controller')
		self.publisher = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
		
		#self.subs0 = self.create_subscription(JointState, '/joint_states', self.cbjs, 10)
  
		# self.subPos = Subscriber(self, Odometry,'/odom/robot_pos')
		self.create_subscription(Odometry, '/odom/robot_pos', self.callback_position, 10)
		self.subJoints = Subscriber(self, JointState, '/joint_states')
		self.subIMU = Subscriber(self, Imu, '/imu/data')

		# Pozisyon verisini diğer veriler ile senkronize etmeye çalışmak kontrolcünün performansını
		# düşürüyor ya da bozuyor. Bu yüzden pozisyon verisini ayrıca kaydediyoruz.
		TimeSynchronizer([
			# self.subPos,
			self.subJoints,
			self.subIMU
			], 
			10
		).registerCallback(self.callback)

		# Declare socket parameters and their default values for communication with RosNet
		self.declare_parameter('host', "127.0.0.1")     # Standard loopback interface address (localhost)
		self.declare_parameter('port', 1256)            # Port to listen on (non-privileged ports are > 1023)

		# Retrieve socket parameters since they can be changed while calling the launch file
		self.host = self.get_parameter("host").value    
		self.port = self.get_parameter("port").value

		# Create a STREAM socket and start listening
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.socket.bind((self.host, self.port))
		self.socket.listen()
		self.get_logger().info("Server listening on {}:{}".format(self.host, self.port))

		self.currTime = 0
		self.currPos = np.zeros(6)
		self.currVel = np.zeros(6)
		self.currTorq = np.zeros(6)
		self.currPose = np.zeros(4)
		self.globalPos = np.zeros(3)
		self.clientMsg = ""
		self.newdata = False

		self.cmd_time = 0
		self.cmd_pos = np.zeros(6)
		self.cmd_kp = np.zeros(6)
		self.cmd_vel = np.zeros(6)
		self.cmd_kd = np.zeros(6)
		self.cmd_tau = np.zeros(6)
		self.cmd_enabled = False

		self.lock = threading.Lock()
		self.exitThread = False

		self.get_logger().info("\n\n---------------------- The node is ready! ----------------------\n\n")

		self.s = 0
		self.posipdis = [0]*6
		self.torque = Float64MultiArray()
		self.torque.data = [0.0]*6   #initial torque data.

		return
		# self.subPos = Subscriber(self, Odometry,'/odom/robot_pos')
		"""
		self.create_subscription(Odometry, '/odom/robot_pos', self.callback_position, 10)
		self.subIMU = Subscriber(self, Imu, '/imu/data')

		# Pozisyon verisini diğer veriler ile senkronize etmeye çalışmak kontrolcünün performansını
		# düşürüyor ya da bozuyor. Bu yüzden pozisyon verisini ayrıca kaydediyoruz.
		TimeSynchronizer([
			# self.subPos,
			self.subJoints,
			self.subIMU
			], 
			10
		).registerCallback(self.callback)

		# Declare socket parameters and their default values for communication with RosNet
		self.declare_parameter('host', "127.0.0.1")     # Standard loopback interface address (localhost)
		self.declare_parameter('port', 1256)            # Port to listen on (non-privileged ports are > 1023)

		# Retrieve socket parameters since they can be changed while calling the launch file
		self.host = self.get_parameter("host").value    
		self.port = self.get_parameter("port").value

		# Create a STREAM socket and start listening
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.socket.bind((self.host, self.port))
		self.socket.listen()
		self.get_logger().info("Server listening on {}:{}".format(self.host, self.port))
		"""

		self.currTime = 0
		self.currPos = np.zeros(6)
		self.currVel = np.zeros(6)
		self.currTorq = np.zeros(6)
		self.currPose = np.zeros(4)
		self.globalPos = np.zeros(3)
		self.clientMsg = ""
		self.newdata = False

		self.posipdis = np.zeros(6)
		self.posipdo = np.zeros(6)

		self.cmd_time = 0
		self.cmd_pos = np.zeros(6)
		self.cmd_kp = np.zeros(6)
		self.cmd_vel = np.zeros(6)
		self.cmd_kd = np.zeros(6)
		self.cmd_tau = np.zeros(6)
		self.cmd_enabled = False

		self.lock = threading.Lock()
		self.exitThread = False

		self.get_logger().info("\n\n---------------------- The node is ready! ----------------------\n\n")
  
	def cbjs(s, m:JointState):
		print("cbjs called")
		"""
		with s.lock:
			s.currPos = constrain_angle(np.array([*m.position]))
			input("callback:")
  """	
	def callback(self,
				#  msgOdom: Odometry,
				 msgJoint: JointState,
				 msgIMU: Imu
				):
		print(f"entered callback")
		self.currPos = constrain_angle(np.array([*msgJoint.position]))
		self.currVel = np.array([*msgJoint.velocity])


		l1i=0
		l2i=2
		l3i=4
		r1i=1
		r2i=3
		r3i=5
		
		self.torop = [0]*6
		
		self.posipdiso = self.posipdis
		self.posipdis = [posipd(pos) for pos in self.currPos]
			
		c="g2"
		match c:
			case "sd":
				for i in range(6):
					self.mt(i, 0)
			case "g1":
				for i in range(6):
					self.torop[i] = ((self.posipd[i] - self.posipdo[i]) != 1) * 1
			case "g2":    
				self.mt(l2i, 2)
				self.mt(r2i, 2)

				mt = 0
				match self.s:
					case 0:
						mt += self.mt(l1i, 0)
						mt += self.mt(r1i, 0)
						mt += self.mt(l3i, 0)
						mt += self.mt(r3i, 0)
						if mt<3:
							self.s = 1
					case 1:
						mt += self.mt(l1i, -2)
						mt += self.mt(r1i, -2)
						mt += self.mt(l3i, -2)
						mt += self.mt(r3i, -2)
						if mt<3:
							self.s = 0
			case "tr":
				self.mt(l2i, 2)
				self.mt(r2i, 2)
				
				mt=0
				match self.s:
					case 0:
						mt += self.mt(l1i, 0)
						mt += self.mt(r1i, 0)
						mt += self.mt(l2i, 0)
						mt += self.mt(r2i, 0)
						mt += self.mt(l3i, 0)
						mt += self.mt(r3i, 0)
						if mt<3:
							self.s = 1
					case 1:
						mt += self.cveloci(r1i, -1)
						mt += self.cveloci(r3i, -1)
						mt += self.cveloci(l1i, 1)
						mt += self.cveloci(l3i, 1)
		
		self.torque.data = [float(tor) for tor in self.torop]
		self.publisher.publish(self.torque)

	def callback_position(self, msg):
		with self.lock:
			self.globalPos = np.array([
				msg.pose.pose.position.x,
				msg.pose.pose.position.y,
				msg.pose.pose.position.z
			])

		"""
	def callback(self,
				#  msgOdom: Odometry,
				 msgJoint: JointState,
				 msgIMU: Imu
				):
		
		with self.lock:
			self.newdata = True
			self.currTime = msgJoint.header.stamp.sec + msgJoint.header.stamp.nanosec/1e9
			self.currPos = constrain_angle(np.array([*msgJoint.position]))
			self.currVel = np.array([*msgJoint.velocity])
			self.currTorq = np.array([*msgJoint.effort])
			self.currPose = np.array([
				msgIMU.orientation.x,
				msgIMU.orientation.y,
				msgIMU.orientation.z,
				msgIMU.orientation.w
			])

			self.clientMsg = "".join([
				f"t: {self.currTime}, ",
				f"hp: {-self.currPos}, ",
				f"hv: {-self.currVel}, ",
				f"ht: {-self.currTorq}, ",
				f"ro: {self.currPose}, ",
				f"rp: {self.globalPos}"
			])
			self.clientMsg = remove_spaces(self.clientMsg)
			self.clientMsg = self.clientMsg.replace("\n", "")+"\n"
		"""

	def run(self):
		self.g=None
		async def i():
			match rc():
				case " ":   self.g = 1-self.g
		#r(i())
		self.s = 0
  

		
		torque = Float64MultiArray()
		torque.data = [0.0]*6   #initial torque data.

		self.get_logger().info("RUN")
		
		while rclpy.ok():
			try:
				if self.newdata:
					torque.data = [float(tor) for tor in self.ctor()]
					self.publisher.publish(torque)

				rclpy.spin_once(self)
				i = input("write input loop:")
				print(i)

				if self.exitThread:
					break

			except KeyboardInterrupt:
				torque.data = [0.0]*6 
				self.publisher.publish(torque)
				self.finish()
				break
	""" data from simulation
	self.currTime = 0
	self.currPos = np.zeros(6)
	self.currVel = np.zeros(6)
	self.currTorq = np.zeros(6)
	self.currPose = np.zeros(4)
	self.globalPos = np.zeros(3)
	self.clientMsg = ""
	self.newdata = False

	self.cmd_time = 0
	self.cmd_pos = np.zeros(6)
	self.cmd_kp = np.zeros(6)
	self.cmd_vel = np.zeros(6)
	self.cmd_kd = np.zeros(6)
	self.cmd_tau = np.zeros(6)
	self.cmd_enabled = False
	"""
	def ctor(self, td=None):
		"""
		 front
		l3^^r3
		l2!!r2
		l1!!r1
		 back
		"""
		l1i=0
		l2i=2
		l3i=4
		r1i=1
		r2i=3
		r3i=5
		
		self.torop = [0]*6
		
		self.posipdiso = self.posipdis
		self.posipdis = [posipd(pos) for pos in self.currPos]
			
		c="g1"
		match c:
			case "sd":
				for i in range(6):
					self.mt(i, 0)
			case "g1":
				for i in range(6):
					self.torop[i] = ((self.posipd[i] - self.posipdo[i]) != 1) * 1
			case "g2":    
				self.mt(l2i, 2)
				self.mt(r2i, 2)

				mt = 0
				match self.s:
					case 0:
						mt += self.mt(l1i, 0)
						mt += self.mt(r1i, 0)
						mt += self.mt(l3i, 0)
						mt += self.mt(r3i, 0)
						if mt<3:
							self.s = 1
					case 1:
						mt += self.mt(l1i, -2)
						mt += self.mt(r1i, -2)
						mt += self.mt(l3i, -2)
						mt += self.mt(r3i, -2)
						if mt<3:
							self.s = 0
			case "tr":
				self.mt(l2i, 2)
				self.mt(r2i, 2)
				
				mt=0
				match self.s:
					case 0:
						mt += self.mt(l1i, 0)
						mt += self.mt(r1i, 0)
						mt += self.mt(l2i, 0)
						mt += self.mt(r2i, 0)
						mt += self.mt(l3i, 0)
						mt += self.mt(r3i, 0)
						if mt<3:
							self.s = 1
					case 1:
						mt += self.cveloci(r1i, -1)
						mt += self.cveloci(r3i, -1)
						mt += self.cveloci(l1i, 1)
						mt += self.cveloci(l3i, 1)
				""" 
				self.torop[r2i] = self.torop[l3i] = tor*1
				match self.posipd[r3i], self.posipd[r2i]:
					case (-1,0) | (0,1) | (1,-1):
						self.torop[r2i] *= -1
				match self.posipd[r3i], self.posipd[l3i]:
					case (-1,0) | (0,1) | (1,-1):
						self.torop[l3i] *= -1
				self.torop[r3i] = -self.torop[r2i] - self.torop[l3i]
				
				self.torop[r1i] = tor*1
				match self.posipd[r3i], self.posipd[r1i]:
					case (-1,0) | (0,1) | (1,-1):
						self.torop[r1i] *= -1
				self.torop[r3i] += -self.torop[r1i]

				#print(possl)
				for i in range(6):
					self.torop[i] += tor*2
				self.torop[l3i] = 9
				"""
		return self.torop

	def mt(self, lgi, posdis):
		dif = posdif(self.posipdis[lgi], posdis)
		self.torop[lgi] += dif * 8
		return abs(dif)
		print(f"leg id {lgi}: {self.currPos[lgi]} {self.posipdis[lgi]} -> {posdis}, posfid = {posdif(self.posipdis[lgi], posdis)}")

	def cveloci(self, lgi, veloci):
		dif = veloci - self.currVel[lgi]
		self.torop[lgi] += dif * 8
		return abs(dif)
		print(f"leg id {lgi}: {self.currPos[lgi]} {self.posipdis[lgi]} -> {posdis}, posfid = {posdif(self.posipdis[lgi], posdis)}")

def posipd(pos, am=3):
	for s in range(-am, am+1):
		if s*pi*2/(am*2 +1) <= pos <= (s+1)*pi*2/(am*2 +1):
			return s
	print(f"chaeck posipd(), error")
	return am
def posdif(posdf, posdt, am=3):
	s = 2*am + 1
	posd1 = (posdt - posdf) % (s)
	if posd1 <= am:
		return posd1
	else:
		return posd1-s

def sta(posdis, dposd):
	return posdif(posdis, dposd)

def main(args=None):
	rclpy.init(args=args)

	legController = LegController()
	try:
		while 1:
			rclpy.spin_once(legController)
		legController.run()
	except Exception as e:
		print("not spiinninnnggg hahaha")
		print("Got unhandled exception!\n")
		print(e)
	"""
	finally:
		rclpy.shutdown()
	"""


if __name__ == '__main__':
	main()
