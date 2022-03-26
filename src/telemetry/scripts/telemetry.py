#!/usr/bin/env python

import socket as sock
from struct import Struct, pack
import time
from typing import NamedTuple, Tuple        # NOTE: install typing with pip

import rospy
from enum import IntEnum  # NOTE! Install enum34 with pip
from std_msgs.msg import Float32, Header
from sensor_msgs.msg import Joy


class Headers(IntEnum):
    REQUEST_TERMINATE = 0
    ODOMETRY = 1
    ARM_ANGLE = 2
    JOY_INPUT = 3


JoyInput = NamedTuple('JoyInput', [
    ('axes', Tuple[float, float, float, float, float, float]),
    ('buttons', Tuple[bool, bool, bool, bool, bool, bool, bool, bool, bool, bool, bool, bool, bool, bool])
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
def serialize_i32(num):
    return _i32_struct.pack(num)


def serialize_f32(num):
    return _f32_struct.pack(num)


def serialize_f64(num):
    return _f64_struct.pack(num)


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
        for i in range(8):
            factor = pow(2, i)
            if n >= factor:
                n -= factor
                bools.append(True)
            else:
                bools.append(False)
            expected_size -= 1
            if expected_size == 0:
                return bools


# IMPORTANT NOTE
# The following deserialize methods attempt to deserialize as much as they can from the byte stream
# So they all return tuples
def deserialize_i32(data):
    return _i32_struct.unpack(data)


def deserialize_f32(data):
    return _f32_struct.unpack(data)


def deserialize_f64(data):
    return _f64_struct.unpack(data)


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
        data_slice = self.data[0:4*count]
        del self.data[0:4*count]
        return deserialize_i32(data_slice)[0:count]

    def deserialize_f32(self, count=1):
        """
        :param count: Number of floats to deserialize at once
        :return: A tuple of floats
        """
        data_slice = self.data[0:4*count]
        del self.data[0:4*count]
        return deserialize_f32(data_slice)[0:count]

    def deserialize_f64(self, count=1):
        """
        :param count: Number of doubles to deserialize at once
        :return: A tuple of doubles
        """
        data_slice = self.data[0:8*count]
        del self.data[0:8*count]
        return deserialize_f32(data_slice)[0:count]

    def deserialize_joy(self):
        inp = JoyInput(
            self.deserialize_f32(6),
            tuple(deserialize_bool_array(self.data[0:2], 14))
        )
        del self.data[0:2]
        return inp


class LunabaseStream(object):
    """
    Sets up a bidirectional UDP communications channel with Lunabase, and executes callbacks on received packets
    """

    def __init__(self):
        self.broadcast_listener = sock.socket(sock.AF_INET, sock.SOCK_DGRAM, sock.IPPROTO_UDP)
        self.broadcast_listener.setsockopt(sock.SOL_SOCKET, sock.SO_REUSEADDR, 1)
        self.broadcast_listener.setblocking(False)
        self._listening_for_broadcast = False

        self.stream = sock.socket(sock.AF_INET, sock.SOCK_DGRAM)
        self.stream.setblocking(False)
        self._connected_to_lunabase = False

        self.arm_publish = rospy.Publisher("set_arm_angle", Float32, queue_size=10)
        self.joy_publish = rospy.Publisher("telemetry_joy", Joy, queue_size=10)

    def close(self):
        self.stream.close()
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
                addr, port_str = self.broadcast_listener.recv(1024).split(":")
            except sock.error:
                return
            self.stream.connect((addr, int(port_str)))
            self._connected_to_lunabase = True
            self.broadcast_listener = None
            self._listening_for_broadcast = False
            self.stream.send('\x00')
            rospy.loginfo("Connected to" + addr + ":" + port_str)

        if not self._connected_to_lunabase: return
        msg = bytearray(1024)
        try:
            self.stream.recv_into(msg, 1024)
        except sock.error:
            return

        header = msg[0]
        del msg[0]
        if header == Headers.REQUEST_TERMINATE:
            pass
        elif header == Headers.ARM_ANGLE:
            self.arm_publish.publish(deserialize_f32(msg)[0])
        elif header == Headers.JOY_INPUT:
            joy_inp = DeserializationStream(msg).deserialize_joy()
            joy_header = Header()
            joy_header.stamp = rospy.Time.now()
            joy_msg = Joy(header=joy_header, axes=joy_inp.axes, buttons=joy_inp.buttons)
            self.joy_publish.publish(joy_msg)


if __name__ == "__main__":
    rospy.init_node('telemetry')  # tmp name until I officially overwrite telemetry
    stream = LunabaseStream()
    rospy.on_shutdown(stream.close)
    stream.listen_for_broadcast(
        rospy.get_param("multicast_address"),
        int(rospy.get_param("multicast_port"))
    )
    polling_delay = float(rospy.get_param("polling_delay"))
    while not rospy.is_shutdown():
        time.sleep(0.5)
        stream.poll()
