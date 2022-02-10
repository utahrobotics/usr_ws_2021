#!/usr/bin/env python

import socket
import rospy
import json
from std_msgs.msg import Header
from sensor_msgs.msg import Joy

def shutdown_callback():
	global sock
	rospy.logwarn("closing telemetry socket")
	sock.close()

def local_callback(data):
	global pub
	#header = Header()
	#header.stamp = rospy.Time.now()
	#joyMsg = Joy(header=header, axes=data.axes , buttons=data.buttons)
	pub.publish(data)
	

if __name__ == "__main__":
	source = rospy.get_param('/controller_source')
	pub = rospy.Publisher('telemetry_joy', Joy,  queue_size=10)
	rospy.init_node('telemetry')
	UDP_IP = rospy.get_param('/remote_ip')
	UDP_PORT = 4242
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	rospy.on_shutdown(shutdown_callback)

	if source == "local":
		print("local control, using joy node")
		rospy.Subscriber("joy", Joy, local_callback)
		rospy.spin()
		
	else:
		MESSAGE = b"Robot Init"
		sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

		while not rospy.is_shutdown():
			data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
	    		print("received message: %s" % data)
			state = json.loads(data)
			rospy.logwarn(state)
			header = Header()
			header.stamp = rospy.Time.now()
			joyMsg = Joy(header=header, axes=state["axes"] , buttons=state["buttons"])
			pub.publish(joyMsg)


		
		
