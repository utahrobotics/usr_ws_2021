#!/usr/bin/env python

import socket as sock
from struct import pack

import rospy
from enum import IntEnum  # NOTE! Install enum34 with pip
from std_msgs.msg import Float32, Header, Bool
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from motors.msg import HomeMotorManualAction, HomeMotorManualGoal, FakeInitAction, FakeInitGoal
from actionlib import SimpleActionClient
from rosgraph_msgs.msg import Log
from autonomy.msg import DumpAction, DumpGoal

from serde import serialize_odometry, deserialize_f32, JoyInput


class MsgHeaders(IntEnum):
	REQUEST_TERMINATE = 0
	ODOMETRY = 1
	ARM_ANGLE = 2
	JOY_AXIS = 3
	MAKE_AUTONOMOUS = 4
	MAKE_MANUAL = 5
	ECHO = 6
	START_MANUAL_HOME = 7
	CONNECTED = 8
	PING = 9
	ROSOUT = 10  # A rosout message
	SEND_ROSOUT = 11
	DONT_SEND_ROSOUT = 12
	DUMP = 13
	FAKE_INIT = 14
	JOY_BUTTON = 15


def pub_joy(pub, joy):
	joy_header = Header()
	joy_header.stamp = rospy.Time.now()
	pub.publish(
		Joy(header=joy_header, axes=joy.axes, buttons=joy.buttons)
	)


class LunabaseStream(object):
	"""
	Sets up a bidirectional UDP communications channel with Lunabase, and executes callbacks on received packets
	"""
	
	def __init__(self):
		self.broadcast_listener = None
		self.udp_stream = None
		self.tcp_stream = None
		self.setup_sockets()
		
		self._connected_to_lunabase = False
		self._listening_for_broadcast = False
		self.termination_requested = False
		self.is_autonomous = False
		
		self.rosout_sub = rospy.Subscriber("rosout", Log, self.rosout_callback, queue_size=10)
		self.is_sending_rosout = True
		self.odom_sub = rospy.Subscriber("nav_msgs/Odometry", Odometry, self.odom_callback, queue_size=10)
		
		self.fake_init_client = SimpleActionClient("fake_init_as", FakeInitAction)
		self.arm_publish = rospy.Publisher("set_arm_angle", Float32, queue_size=1)
		self.joy_publish = rospy.Publisher("telemetry_joy", Joy, queue_size=1)
		self.autonomy_publish = rospy.Publisher("set_autonomy", Bool, queue_size=10)
		self.manual_home_client = SimpleActionClient("home_motor_manual_as", HomeMotorManualAction)
		self.dump_client = SimpleActionClient("Dump", DumpAction)
		
		timeout = rospy.Duration(3)
		self.manual_home_client.wait_for_server(timeout)
		self.dump_client.wait_for_server(timeout)
		
		self.joy_input = JoyInput()
		self.joy_skip = 5
		self._current_joy_skip = 0
	
	def setup_sockets(self):
		self.broadcast_listener = sock.socket(sock.AF_INET, sock.SOCK_DGRAM, sock.IPPROTO_UDP)
		self.broadcast_listener.setsockopt(sock.SOL_SOCKET, sock.SO_REUSEADDR, 1)
		self.broadcast_listener.setblocking(False)
		
		self.udp_stream = sock.socket(sock.AF_INET, sock.SOCK_DGRAM)
		self.tcp_stream = sock.socket(sock.AF_INET, sock.SOCK_STREAM)
	
	def close(self):
		self.udp_stream.close()
		self.tcp_stream.close()
		if self.broadcast_listener is not None:
			self.broadcast_listener.close()
		self._listening_for_broadcast = False
		self._connected_to_lunabase = False
	
	def listen_for_broadcast(self, addr="224.1.1.1", port=42420):
		self.broadcast_listener.bind(('', port))
		mreq = pack("4sl", sock.inet_aton(addr), sock.INADDR_ANY)
		self.broadcast_listener.setsockopt(sock.IPPROTO_IP, sock.IP_ADD_MEMBERSHIP, mreq)
		self._listening_for_broadcast = True
	
	def direct_connect(self, addr="127.0.0.1", port=42424):
		self.udp_stream.connect((addr, port))
		self.tcp_stream.connect((addr, port + 1))
		self.udp_stream.setblocking(False)
		self.tcp_stream.setblocking(False)
		self.udp_stream.sendall(bytearray([MsgHeaders.CONNECTED]))
		self._connected_to_lunabase = True
		rospy.logwarn("Successfully connected to lunabase")
	
	def rosout_callback(self, msg):
		if not self._connected_to_lunabase or not self.is_sending_rosout: return
		self.tcp_stream.sendall(bytearray([MsgHeaders.ROSOUT, msg.level]) + bytes(msg.msg))
	
	def odom_callback(self, odom):
		if not self._connected_to_lunabase: return
		self.udp_stream.sendall(bytearray([MsgHeaders.ODOMETRY]) + serialize_odometry(odom))
	
	def poll(self):
		if self._listening_for_broadcast:
			try:
				addr, port_str = str(self.broadcast_listener.recv(1024)).split(":")
			except sock.error:
				return
			self.direct_connect(addr, int(port_str))
			self.broadcast_listener = None
			self._listening_for_broadcast = False
		
		if not self._connected_to_lunabase: return
		try:
			msg, _ = self.udp_stream.recvfrom(1024)
			self._handle_message(bytearray(msg))
		except sock.error:
			pass
		
		try:
			self.tcp_stream.fileno()  # method that pings the remote server to check if it is still up
			msg, _ = self.tcp_stream.recvfrom(1024)
			self._handle_message(bytearray(msg))
		except sock.error:
			pass
		
		# Echo the last joy message if we haven't received one in a while
		self._current_joy_skip -= 1
		if self._current_joy_skip == 0:
			self._current_joy_skip = self.joy_skip
			pub_joy(self.joy_publish, self.joy_input)
	
	def _handle_message(self, msg):
		if len(msg) == 0:
			# Last test of this was unsuccessful
			rospy.logwarn("Remote base has closed connection to us, reconnecting...")
			self._connected_to_lunabase = False
			self.close()
			self.setup_sockets()
			self._listening_for_broadcast = True
			return
		
		header = msg[0]
		del msg[0]
		if header == MsgHeaders.REQUEST_TERMINATE:
			# TODO Add method to stop the bot
			rospy.logwarn("Remote base wants us to terminate")
			self.termination_requested = True
		
		elif header == MsgHeaders.ARM_ANGLE:
			self.arm_publish.publish(deserialize_f32(msg)[0])
		
		elif header == MsgHeaders.JOY_AXIS:
			if self.is_autonomous:
				rospy.logwarn("Ignoring joy axis!")
				return
			self._current_joy_skip = self.joy_skip
			pub_joy(self.joy_publish, self.joy_input.deserialize_joy_axis(msg[0]))
		
		elif header == MsgHeaders.JOY_BUTTON:
			if self.is_autonomous:
				rospy.logwarn("Ignoring joy button!")
				return
			self._current_joy_skip = self.joy_skip
			pub_joy(self.joy_publish, self.joy_input.deserialize_joy_button(msg[0]))
		
		elif header == MsgHeaders.MAKE_AUTONOMOUS:
			if self.is_autonomous:
				rospy.logwarn("Is already autonomous!")
				return
			self.autonomy_publish.publish(Bool(data=True))
			self.is_autonomous = True
			rospy.logwarn("Received MAKE_AUTONOMOUS")
			self.tcp_stream.sendall(bytearray([MsgHeaders.ECHO, MsgHeaders.MAKE_AUTONOMOUS]))
		
		elif header == MsgHeaders.MAKE_MANUAL:
			if not self.is_autonomous:
				rospy.logwarn("Is already manual!")
				return
			self.autonomy_publish.publish(Bool(data=False))
			self.is_autonomous = False
			rospy.logwarn("Received MAKE_MANUAL")
			self.tcp_stream.sendall(bytearray([MsgHeaders.ECHO, MsgHeaders.MAKE_MANUAL]))
		
		elif header == MsgHeaders.START_MANUAL_HOME:
			goal = HomeMotorManualGoal()
			goal.motor = msg[0]
			self.manual_home_client.send_goal(goal)
			rospy.logwarn("manually homing! ;-)")
		
		elif header == MsgHeaders.SEND_ROSOUT:
			if self.is_sending_rosout:
				rospy.logwarn("We are already sending rosout!")
				return
			self.is_sending_rosout = True
			rospy.logwarn("Is sending rosout!")
		
		elif header == MsgHeaders.DONT_SEND_ROSOUT:
			if not self.is_sending_rosout:
				rospy.logwarn("We are already not sending rosout!")
				return
			self.is_sending_rosout = False
			rospy.logwarn("Is not sending rosout!")
		
		elif header == MsgHeaders.DUMP:
			if self.is_autonomous:
				rospy.logwarn("Cannot dump while autonomous")
				return
			self.is_autonomous = True
			self.tcp_stream.sendall(bytearray([MsgHeaders.MAKE_AUTONOMOUS]))
			self.dump_client.send_goal(DumpGoal())
			self.dump_client.wait_for_result()
			self.tcp_stream.sendall(bytearray([MsgHeaders.MAKE_MANUAL]))
			self.is_autonomous = False
		
		elif header == MsgHeaders.FAKE_INIT:
			if self.is_autonomous:
				rospy.logwarn("Cannot fake init while autonomous")
				return
			self.is_autonomous = True
			self.tcp_stream.sendall(bytearray([MsgHeaders.MAKE_AUTONOMOUS]))
			goal = FakeInitGoal()
			goal.goal = True
			self.fake_init_client.send_goal(goal)
			self.fake_init_client.wait_for_result()
			self.tcp_stream.sendall(bytearray([MsgHeaders.MAKE_MANUAL]))
			self.is_autonomous = False
		
		else:
			raise Exception("Unrecognized header!: " + str(header))


if __name__ == "__main__":
	rospy.init_node('telemetry')
	if rospy.has_param("/controller_source"):
		param = rospy.get_param("/controller_source")
		if param == "local":
			rospy.logwarn("local control, using joy node")
			pub = rospy.Publisher('telemetry_joy', Joy, queue_size=10)
			rospy.Subscriber("joy", Joy, pub.publish)
			rospy.spin()
			raise SystemExit
		elif param == "remote":
			pass
		else:
			raise ValueError("Unexpected value for /controller_source: " + rospy.get_param("/controller_source"))
	
	stream = LunabaseStream()
	rospy.on_shutdown(stream.close)
	
	if not rospy.has_param("isAutonomous"):
		raise ValueError("isAutonomous is not set. Please add it")
	
	stream.is_autonomous = bool(rospy.get_param("isAutonomous"))
	
	if rospy.has_param("remote_ip"):
		if not rospy.has_param("remote_port"):
			raise KeyError("There is a remote_ip in the launch file, but not a remote_port. Please add it")
		
		addr = rospy.get_param("remote_ip")
		port = rospy.get_param("remote_port")
		rospy.logwarn("Using direct connection to lunabase at " + addr + ":" + str(port))
		stream.direct_connect(
			addr,
			int(port)
		)
	
	else:
		rospy.logwarn("Using broadcasting to discover lunabase")
		stream.listen_for_broadcast(
			rospy.get_param("multicast_address"),
			int(rospy.get_param("multicast_port"))
		)
	
	polling_rate = rospy.get_param("polling_rate")
	rate = rospy.Rate(polling_rate)
	while not rospy.is_shutdown() and not stream.termination_requested:
		rate.sleep()
		stream.poll()
	rospy.logwarn("Polling loop has ended")
