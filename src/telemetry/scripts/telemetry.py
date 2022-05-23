#!/usr/bin/env python

import socket as sock
from struct import pack

import rospy
from enum import IntEnum  # NOTE! Install enum34 with pip
from std_msgs.msg import Float32, Header, Bool
from sensor_msgs.msg import Joy, CompressedImage
from nav_msgs.msg import Odometry
from motors.msg import HomeMotorManualAction, HomeMotorManualGoal, FakeInitAction, FakeInitGoal
from autonomy.msg import StartMachineAction, StartMachineGoal, StartMachineFeedback, StartMachineResult, InitializeAction, InitializeGoal
from actionlib import SimpleActionClient
from rosgraph_msgs.msg import Log
from autonomy.msg import DumpAction, DumpGoal, DigAction, DigGoal
import roslaunch
import tf
from geometry_msgs.msg import Pose, PoseWithCovariance, TwistStamped, Twist

from serde import serialize_odometry, JoyInput, serialize_f32


class MsgHeaders(IntEnum):
	PING = 0
	ODOMETRY = 1
	ARM_ANGLE = 2
	JOY_AXIS = 3
	INITIATE_AUTONOMY_MACHINE = 4
	MAKE_MANUAL = 5
	START_MANUAL_HOME = 6
	CONNECTED = 7
	ROSOUT = 8  # A rosout message
	SEND_ROSOUT = 9
	DONT_SEND_ROSOUT = 10
	DUMP = 11
	INIT_BOT = 12
	JOY_BUTTON = 13
	VID_STREAM = 14
	SEND_STREAM = 15
	DONT_SEND_STREAM = 16
	DIG = 17


def pub_joy(pub, joy):
	print(joy.axes)
	joy_header = Header()
	joy_header.stamp = rospy.Time.now()
	pub.publish(
		Joy(header=joy_header, axes=joy.axes, buttons=joy.buttons)
	)


class Timer(object):
	"""Helper class to keep track of time"""
	def __init__(self, duration):
		self.duration = duration
		self._current_duration = duration
	
	def elapse(self, delta):
		self._current_duration -= delta
		if self._current_duration <= 0:
			self._current_duration = self.duration
			return True
		return False
	
	def reset(self):
		self._current_duration = self.duration


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
		
		self.imu_sub = rospy.Subscriber("sensors/imu/vel", TwistStamped, self.imu_vel_callback)
		self.last_twist = Twist()
		self.rosout_sub = rospy.Subscriber("rosout", Log, self.rosout_callback, queue_size=10)
		self.is_sending_rosout = True
		self.arm_angle_sub = rospy.Subscriber('/sensors/angleSensor/angle', Float32, self.arm_angle_callback)
		self.arm_angle = 0.0
		
		self._is_streaming_vid = True
		self.webcam_sub = rospy.Subscriber("/camera/compressed", CompressedImage, self.img_callback, queue_size=10)
		self.frame_id = 0

		# self.arm_publish = rospy.Publisher("set_arm_angle", Float32, queue_size=1)
		self.joy_publish = rospy.Publisher("telemetry_joy", Joy, queue_size=1)
		self.autonomy_publish = rospy.Publisher("set_autonomy", Bool, queue_size=10)
		
		self.init_client = SimpleActionClient("fake_init_as", FakeInitAction)
		self.manual_home_client = SimpleActionClient("home_motor_manual_as", HomeMotorManualAction)
		self.dump_client = SimpleActionClient("Dump", DumpAction)
		self.dig_client = SimpleActionClient("Dig", DigAction)
		self.start_machine_client = SimpleActionClient("start_machine_as", StartMachineAction)
		
		timeout = rospy.Duration(3)
		self.manual_home_client.wait_for_server(timeout)
		self.dump_client.wait_for_server(timeout)
		self.start_machine_client.wait_for_server(timeout)
		
		self.tf_listener = tf.TransformListener()
		self.odom_timer = Timer(1)
		
		self.joy_input = JoyInput()
		self.joy_timer = Timer(0.5)
		
		self._last_ip = ""
		self._last_port = 0
	
	def setup_sockets(self):
		self.broadcast_listener = sock.socket(sock.AF_INET, sock.SOCK_DGRAM, sock.IPPROTO_UDP)
		self.broadcast_listener.setsockopt(sock.SOL_SOCKET, sock.SO_REUSEADDR, 1)
		self.broadcast_listener.setblocking(False)
		self.joy_input = JoyInput()
		
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
		self._last_ip = addr
		self._last_port = port
		rospy.logwarn("Successfully connected to lunabase")
	
	def imu_vel_callback(self, twist_stamped):
		self.last_twist = twist_stamped.twist
	
	def img_callback(self, msg):
		if not self._connected_to_lunabase or not self._is_streaming_vid: return
		self.udp_stream.sendall(bytearray([MsgHeaders.SEND_STREAM, self.frame_id]) + bytearray(msg.data))
		self.frame_id += 1
		if self.frame_id == 256:
			self.frame_id = 0
	
	def rosout_callback(self, msg):
		if not self._connected_to_lunabase or not self.is_sending_rosout: return
		self.tcp_stream.sendall(bytearray([MsgHeaders.ROSOUT, msg.level]) + bytes(msg.msg))
	
	def arm_angle_callback(self, msg):
		self.arm_angle = msg.data
	
	def send_odom(self, odom):
		#self.udp_stream.sendall(bytearray([MsgHeaders.ODOMETRY]) + serialize_odometry(odom))
		self.udp_stream.sendall(bytearray([MsgHeaders.ARM_ANGLE]) + serialize_f32(self.arm_angle))
	
	def poll(self, delta):
		if self._listening_for_broadcast:
			try:
				addr, port_str = str(self.broadcast_listener.recv(1024)).split(":")
				self.direct_connect(addr, int(port_str))
			except sock.error:
				return
			self.broadcast_listener = None
			self._listening_for_broadcast = False
		
		if not self._connected_to_lunabase:
			try:
				self.direct_connect(self._last_ip, self._last_port)
			except sock.error:
				return
		
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
		if self.joy_timer.elapse(delta):
			pub_joy(self.joy_publish, self.joy_input)
		
		if self.odom_timer.elapse(delta) and not rospy.get_param("/isAutonomous"):
			try:
				origin, rotation = self.tf_listener.lookupTransform("/map", "/base_link", rospy.Time(0))
				self.send_odom(self._construct_odom(self.last_twist, origin, rotation))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				pass
	
	def _handle_message(self, msg):
		if len(msg) == 0:
			# Last test of this was unsuccessful
			rospy.logwarn("Remote base has closed connection to us, reconnecting...")
			self._connected_to_lunabase = False
			self.close()
			self.setup_sockets()
			return
		
		header = msg[0]
		del msg[0]
		if header == MsgHeaders.PING:
			rospy.logwarn("Pinged")
			self.tcp_stream.sendall(bytearray([MsgHeaders.PING]))
		
		elif header == MsgHeaders.ARM_ANGLE:
			rospy.logwarn("Unimplemented arm angle header!")
			# self.arm_publish.publish(deserialize_f32(msg)[0])
		
		elif header == MsgHeaders.JOY_AXIS:
			if rospy.get_param("/isAutonomous"):
				rospy.logwarn("Ignoring joy axis!")
				return
			self.joy_timer.reset()
			print(bytes(list(msg)))
			self.joy_input.deserialize_joy_axis(msg[0])
			pub_joy(self.joy_publish, self.joy_input)
		
		elif header == MsgHeaders.JOY_BUTTON:
			if rospy.get_param("/isAutonomous"):
				rospy.logwarn("Ignoring joy button!")
				return
			self.joy_timer.reset()
			self.joy_input.deserialize_joy_button(msg[0])
			pub_joy(self.joy_publish, self.joy_input)
		
		elif header == MsgHeaders.MAKE_MANUAL:
			if not rospy.get_param("/isAutonomous"):
				rospy.logwarn("Is already manual!")
				return
			self.autonomy_publish.publish(Bool(data=False))
			rospy.set_param("/isAutonomous", False)
			rospy.logwarn("Received MAKE_MANUAL")
		
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
			if rospy.get_param("/isAutonomous"):
				rospy.logwarn("Cannot dump while autonomous")
				return
			rospy.set_param("/isAutonomous", True)
			self.tcp_stream.sendall(bytearray([MsgHeaders.INITIATE_AUTONOMY_MACHINE]))
			self.dump_client.send_goal(DumpGoal())
			self.dump_client.wait_for_result()
			self.tcp_stream.sendall(bytearray([MsgHeaders.MAKE_MANUAL]))
			rospy.set_param("/isAutonomous", False)
		
		elif header == MsgHeaders.DIG:
			if rospy.get_param("/isAutonomous"):
				rospy.logwarn("Cannot dig while autonomous")
				return
			rospy.set_param("/isAutonomous", True)
			self.tcp_stream.sendall(bytearray([MsgHeaders.INITIATE_AUTONOMY_MACHINE]))
			self.dig_client.send_goal(DigGoal())
			self.dump_client.wait_for_result()
			self.tcp_stream.sendall(bytearray([MsgHeaders.MAKE_MANUAL]))
			rospy.set_param("/isAutonomous", False)
		
		elif header == MsgHeaders.INIT_BOT:
			if rospy.get_param("/isAutonomous"):
				rospy.logwarn("Cannot init bot while autonomous")
				return
			rospy.set_param("/isAutonomous", True)
			self.tcp_stream.sendall(
				bytearray(
			[MsgHeaders.INITIATE_AUTONOMY_MACHINE]))
			goal = InitializeGoal()
			self.init_client.send_goal(goal)
			self.init_client.wait_for_result()
			self.tcp_stream.sendall(bytearray([MsgHeaders.MAKE_MANUAL]))
			rospy.set_param("/isAutonomous", False)

		elif header == MsgHeaders.INITIATE_AUTONOMY_MACHINE:
			if rospy.get_param("/isAutonomous"):
				rospy.logwarn("Already autonomous!")
				return
			rospy.set_param("/isAutonomous", True)
			goal = StartMachineGoal()
			self.start_machine_client.send_goal(goal)
			self.start_machine_client.wait_for_result()
			rospy.logwarn("state machine finished")
			rospy.set_param("/isAutonomous", False)
		
		elif header == MsgHeaders.SEND_STREAM:
			if self._is_streaming_vid:
				rospy.logwarn("Already streaming video")
				return
			self._is_streaming_vid = True
			rospy.logwarn("Streaming video")
		
		elif header == MsgHeaders.DONT_SEND_STREAM:
			if not self._is_streaming_vid:
				rospy.logwarn("Already not streaming video")
				return
			self._is_streaming_vid = False
			rospy.logwarn("Not streaming video")
		
		else:
			raise Exception("Unrecognized header!: " + str(header))
	
	@staticmethod
	def _construct_odom(twist, position, orientation):
		pose = Pose()
		pose.position = position
		pose.orientation = orientation
		
		odom = Odometry()
		odom.pose = PoseWithCovariance()
		odom.pose.pose = pose
		
		odom.twist.twist = twist
		
		return odom


if __name__ == "__main__":
	joy_process = None
	
	rospy.init_node('telemetry')
	if rospy.has_param("/controller_source"):
		param = rospy.get_param("/controller_source")
		if param == "local":
			rospy.logwarn("local control, using joy node")
			
			# Start Joy node
			launcher = roslaunch.scriptapi.ROSLaunch()
			launcher.start()
			joy_process = launcher.launch(roslaunch.core.Node("joy", "joy_node"))
			
			pub = rospy.Publisher('telemetry_joy', Joy,  queue_size=10)
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

	if rospy.has_param("remote_ip"):
		if rospy.has_param("remote_port"):
			port = int(rospy.get_param("remote_port"))
		else:
			port = 42420
			rospy.logwarn("There is a remote_ip in the launch file, but not a remote_port. Defaulting to 42424")

		addr = rospy.get_param("remote_ip")
		rospy.logwarn("Using direct connection to lunabase at " + addr + ":" + str(port))
		
		while True:
			try:
				stream.direct_connect(
					addr,
					int(port)
				)
				break
			except sock.error:
				pass

	else:
		rospy.logwarn("Using broadcasting to discover lunabase")
		
		if rospy.has_param("multicast_address"):
			addr = rospy.get_param("multicast_address")
		else:
			rospy.logwarn("multicast_address was not set, defaulting to 224.1.1.1")
			addr = "224.1.1.1"
		
		if rospy.has_param("multicast_port"):
			port = int(rospy.get_param("multicast_port"))
		
		else:
			rospy.logwarn("multicast_port was not set, defaulting to 42420")
			port = 42420
		
		stream.listen_for_broadcast(
			addr,
			port
		)

	polling_rate = rospy.get_param("polling_rate")
	delta = 1.0 / polling_rate
	rate = rospy.Rate(polling_rate)
	while not rospy.is_shutdown():
		rate.sleep()
		stream.poll(delta)
	rospy.logwarn("Polling loop has ended")
	
	if joy_process is not None:
		joy_process.stop()
