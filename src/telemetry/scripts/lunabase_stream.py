import socket as sock
from struct import Struct, pack
import time
from rospy import Publisher, init_node, on_shutdown
from enum import IntEnum
from std_msgs.msg import Float32


class Headers(IntEnum):
    REQUEST_TERMINATE = 0
    ODOMETRY = 1
    ARM_ANGLE = 2


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


def serialize_i32(num):
    return _i32_struct.pack(num)


def serialize_f32(num):
    return _f32_struct.pack(num)


def serialize_f64(num):
    return _f64_struct.pack(num)


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

    def deserialize_i32(self):
        data_slice = self.data[0:4]
        del self.data[0:4]
        return deserialize_i32(data_slice)

    def deserialize_f32(self):
        data_slice = self.data[0:4]
        del self.data[0:4]
        return deserialize_f32(data_slice)

    def deserialize_f64(self):
        data_slice = self.data[0:8]
        del self.data[0:8]
        return deserialize_f32(data_slice)


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

        self.arm_publish = Publisher("set_arm_angle", Float32, queue_size=10)

    def close(self):
        self.stream.close()
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
            print("Connected to", addr, port_str)

        if not self._connected_to_lunabase: return
        msg = bytearray()
        try:
            self.stream.recv_into(msg, 1024)
        except sock.error:
            return

        header = msg[0]
        if header == Headers.REQUEST_TERMINATE:
            pass
        elif header == Headers.ARM_ANGLE:
            self.arm_publish.publish(deserialize_f32(msg))


if __name__ == "__main__":
    print(get_my_ip())
    init_node('lunabase_stream')        # tmp name until I officially overwrite telemetry
    stream = LunabaseStream()
    on_shutdown(stream.close)
    stream.listen_for_broadcast()
    while not rospy.is_shutdown():
        time.sleep(0.5)
        stream.poll()
