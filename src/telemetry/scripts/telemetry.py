#!/usr/bin/env python

import socket as sock
from struct import Struct, pack
import time
from typing import NamedTuple, Tuple, List  # NOTE: install typing with pip

import rospy
from enum import IntEnum  # NOTE! Install enum34 with pip
from std_msgs.msg import Float32, Header, Bool
from sensor_msgs.msg import Joy
from motors.msg import HomeMotorManualAction, HomeMotorManualGoal
from actionlib import SimpleActionClient


class MsgHeaders(IntEnum):
    REQUEST_TERMINATE = 0
    ODOMETRY = 1
    ARM_ANGLE = 2
    JOY_INPUT = 3
    MAKE_AUTONOMOUS = 4
    MAKE_MANUAL = 5
    ECHO = 6
    START_MANUAL_HOME = 7
    CONNECTED = 8


JoyInput = NamedTuple('JoyInput', [
    ('axes', List[float]),
    ('buttons', List[bool])
])


_MY_IP = ""


def get_my_ip():
    global _MY_IP
    if len(_MY_IP) == 0:
        s = sock.socket(sock.AF_INET, sock.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        _MY_IP = s.getsockname()[0]
        s.close()
    return _MY_IP


_i32_struct = Struct("<i")
_f32_struct = Struct("<f")
_f64_struct = Struct("<d")
# The following serialize methods can accept a tuple of numbers; All the numbers will be serialized contiguously
serialize_i32 = _i32_struct.pack
serialize_f32 = _f32_struct.pack
serialize_f64 = _f64_struct.pack


def serialize_bool_array(bools):
    data = bytearray()
    size = len(bools)
    iterations = size // 8

    for i in range(iterations):
        idx = i * 8
        data.append(
            bools[idx] * 1 +
            bools[idx + 1] * 2 +
            bools[idx + 2] * 4 +
            bools[idx + 3] * 8 +
            bools[idx + 4] * 16 +
            bools[idx + 5] * 32 +
            bools[idx + 6] * 64 +
            bools[idx + 7] * 128
        )

    if size % 8 != 0:
        offset = iterations * 8
        num = 0
        for i in range(0, size - offset):
            if bools[i + offset]:
                num += pow(2, i)
        data.append(num)

    return data


def deserialize_bool_array(data, expected_size):
    bools = []
    for n in data:
        byte = []
        for i in range(8):
            factor = pow(2, 7 - i)
            if n >= factor:
                n -= factor
                byte.append(True)
            else:
                byte.append(False)
        byte.reverse()
        for b in byte:
            bools.append(b)
            expected_size -= 1
            if expected_size == 0:
                return bools


# IMPORTANT NOTE
# The following deserialize methods attempt to deserialize as much as they can from the byte stream
# So they all return tuples
deserialize_i32 = _i32_struct.unpack
_deserialize_f32 = _f32_struct.unpack
deserialize_f64 = _f64_struct.unpack


class DeserializationStream(object):
    """
    A helper class for deserializing byte arrays with more than one serialized element
    Every deserialize call removes data from the beginning of the byte array
    """

    def __init__(self, data):
        self.data = data

    def remaining_bytes(self):
        return len(self.data)

    def deserialize_i32(self, count=1):
        """
        :param count: Number of ints to deserialize at once
        :return: A tuple of ints
        """
        out = []
        for i in range(count):
            out.append(deserialize_i32(self.data[i * 4: (i + 1) * 4])[0])
            del self.data[i * 4: (i + 1) * 4]
        return out

    def deserialize_f32(self, count=1):
        """
        :param count: Number of floats to deserialize at once
        :return: A tuple of floats
        """
        out = []
        for i in range(count):
            out.append(_deserialize_f32(self.data[i * 4: (i + 1) * 4])[0])
        del self.data[0: count * 4]
        return out

    def deserialize_f64(self, count=1):
        """
        :param count: Number of doubles to deserialize at once
        :return: A tuple of doubles
        """
        out = []
        for i in range(count):
            out.append(deserialize_f64(self.data[i * 8: (i + 1) * 8])[0])
            del self.data[i * 8: (i + 1) * 8]
        return out

    def deserialize_joy(self):
        inp = JoyInput(
            self.deserialize_f32(8),
            deserialize_bool_array(self.data[0:2], 10)
        )
        del self.data[0:2]
        return inp


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

        self.arm_publish = rospy.Publisher("set_arm_angle", Float32, queue_size=1)
        self.joy_publish = rospy.Publisher("telemetry_joy", Joy, queue_size=1)
        self.autonomy_publish = rospy.Publisher("set_autonomy", Bool, queue_size=10)
        self.manual_home_client = SimpleActionClient("home_motor_manual_as", HomeMotorManualAction)

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

    def poll(self):
        if self._listening_for_broadcast:
            try:
                addr, port_str = str(self.broadcast_listener.recv(1024)).split(":")
            except sock.error:
                return
            self.udp_stream.connect((addr, int(port_str)))
            self.tcp_stream.connect((addr, int(port_str) + 1))
            self.udp_stream.setblocking(False)
            self.tcp_stream.setblocking(False)
            self.tcp_stream.sendall(bytes("hello"))
            self.udp_stream.sendall(bytes("hello"))
            self._connected_to_lunabase = True
            self.broadcast_listener = None
            self._listening_for_broadcast = False
            rospy.loginfo("Connected to" + addr + ":" + port_str)

        if not self._connected_to_lunabase: return
        try:
            #msg = bytearray(4096)
            msg, _ = self.udp_stream.recvfrom(1024)
            self._handle_message(bytearray(msg))
        except sock.error:
            pass

        try:
            #msg = bytearray(4096)
            msg, _ = self.tcp_stream.recvfrom(1024)
            self._handle_message(bytearray(msg))
        except sock.error:
            pass

    def _handle_message(self, msg):
        if len(msg) == 0:
            rospy.logwarn("Remote base has closed connection to us, reconnecting...")
            self._connected_to_lunabase = False
            self.close()
            self.setup_sockets()
            self._listening_for_broadcast = True
            return

        header = msg[0]
        del msg[0]
        if header == MsgHeaders.REQUEST_TERMINATE:
            rospy.logwarn("Remote base wants us to terminate")

        elif header == MsgHeaders.ARM_ANGLE:
            self.arm_publish.publish(deserialize_f32(msg)[0])

        elif header == MsgHeaders.JOY_INPUT:
            joy_inp = DeserializationStream(msg).deserialize_joy()
            rospy.logwarn(joy_inp)
            joy_header = Header()
            joy_header.stamp = rospy.Time.now()
            self.joy_publish.publish(Joy(header=joy_header, axes=joy_inp.axes, buttons=joy_inp.buttons))

        elif header == MsgHeaders.MAKE_AUTONOMOUS:
            self.autonomy_publish.publish(Bool(data=True))

        elif header == MsgHeaders.MAKE_MANUAL:
            self.autonomy_publish.publish(Bool(data=False))

        elif header == MsgHeaders.START_MANUAL_HOME:
            goal = HomeMotorManualGoal()
            goal.motor = msg[0]
            self.manual_home_client.send_goal(goal)
            rospy.logwarn("manually homing! ;-)")

        else:
            raise Exception("Unrecognized header!: " + str(header))


if __name__ == "__main__":
    rospy.init_node('telemetry')
    stream = LunabaseStream()
    rospy.on_shutdown(stream.close)
    stream.listen_for_broadcast(
        rospy.get_param("multicast_address"),
        int(rospy.get_param("multicast_port"))
    )
    polling_delay = float(rospy.get_param("polling_delay"))
    rate = rospy.Rate(25)       # 15 Hz
    while not rospy.is_shutdown():
        rate.sleep()
        stream.poll()
