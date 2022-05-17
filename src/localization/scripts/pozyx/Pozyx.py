#!/usr/bin/env python
"""
The Pozyx ready to localize tutorial (c) Pozyx Labs
Please read the tutorial that accompanies this sketch:
https://www.pozyx.io/Documentation/Tutorials/ready_to_localize/Python

This tutorial requires at least the contents of the Pozyx Ready to Localize kit. It demonstrates the positioning capabilities
of the Pozyx device both locally and remotely. Follow the steps to correctly set up your environment in the link, change the
parameters and upload this sketch. Watch the coordinates change as you move your device around!
"""
import math
from time import sleep
import rospy
import actionlib
from localization.msg import GetPoseAction, GetPoseFeedback, GetPoseResult
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

from pypozyx import (POZYX_POS_ALG_UWB_ONLY, POZYX_3D, Coordinates, POZYX_SUCCESS, PozyxConstants, version,
                     DeviceCoordinates, PozyxSerial, get_first_pozyx_serial_port, SingleRegister, DeviceList, PozyxRegisters, Data, SensorData)
from pypozyx.structures import device_information
from pypozyx.tools.version_check import perform_latest_version_check


class ReadyToLocalize(object):
    """Continuously calls the Pozyx positioning function and prints its position."""

    def __init__(self, pozyx, osc_udp_client, anchors, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000, remote_id=None):
        self.pozyx = pozyx
        self.osc_udp_client = osc_udp_client

        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.remote_id = remote_id
        self.lastPose = None;

    def setup(self):
        rospy.init_node('pozyx')
        self.posePub = rospy.Publisher('sensors/pozyx/pose', PoseWithCovarianceStamped, queue_size=10)
        self.imuPub = rospy.Publisher('sensors/pozyx/imu', Imu, queue_size=10)

        self.setGetPoseServer = actionlib.SimpleActionServer(
            "get_pose_as", GetPoseAction, execute_cb=self.getPose_cb, auto_start=False)

        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX POSITIONING V{} -------------".format(version))
        print("")
        print("- System will manually configure tag")
        print("")
        print("- System will auto start positioning")
        print("")
        if self.remote_id is None:
            self.pozyx.printDeviceInfo(self.remote_id)
        else:
            for device_id in [None, self.remote_id]:
                self.pozyx.printDeviceInfo(device_id)
        print("")
        print("------------POZYX POSITIONING V{} -------------".format(version))
        print("")

        self.setAnchorsManual(save_to_flash=False)
        self.printPublishConfigurationResult()

    def getPose_cb(self, goal):
        result = GetPoseResult()
        result.pose = self.lastPose
        self.setGetPoseServer.set_succeeded(result)


    def loop(self):
        """Performs positioning and displays/exports the results."""
        position = Coordinates()
        sensors = SensorData()
        status = self.pozyx.doPositioning(
            position, self.dimension, self.height, self.algorithm, remote_id=self.remote_id)
        if status == POZYX_SUCCESS:
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = rospy.get_rostime()
            pose.header.frame_id = "map"
            pose.pose.pose.position.x = position.x/1000.0
            pose.pose.pose.position.y = position.y/1000.0
            pose.pose.pose.position.z = position.z/1000.0
            self.lastPose = pose
            self.posePub.publish(pose)
            self.printPublishPosition(position)
        else:
            self.printPublishErrorCode("positioning")

        status = self.pozyx.getAllSensorData(sensors, remote_id=self.remote_id)
        if status == POZYX_SUCCESS:
            imuMsg = Imu()

            header = Header()
            header.stamp = rospy.Time.now()

            angular_velocity = Vector3()
            angular_velocity.x = sensors.angular_vel.x * math.pi / 180.0 #d/s to rad/s
            angular_velocity.y = sensors.angular_vel.y * math.pi / 180.0 #d/s to rad/s
            angular_velocity.z = sensors.angular_vel.z * math.pi / 180.0 #d/s to rad/s
            angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

            linear_acceleration = Vector3()
            linear_acceleration.x = (sensors.linear_acceleration.x/1000.0) * 9.8066 #mg to m/s^2
            linear_acceleration.y = (sensors.linear_acceleration.y/1000.0) * 9.8066 #mg to m/s^2
            linear_acceleration.z = (sensors.linear_acceleration.z/1000.0) * 9.8066 #mg to m/s^2
            linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

            orientation_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]

            imuMsg.header = header
            imuMsg.angular_velocity = angular_velocity
            imuMsg.angular_velocity_covariance = angular_velocity_covariance
            imuMsg.linear_acceleration = linear_acceleration
            imuMsg.linear_acceleration_covariance = linear_acceleration_covariance
            imuMsg.orientation_covariance = orientation_covariance

            self.imuPub.publish(imuMsg)
        else:
            self.printPublishErrorCode("sensors")

    def printPublishPosition(self, position):
        """Prints the Pozyx's position and possibly sends it as a OSC packet"""
        network_id = self.remote_id
        if network_id is None:
            network_id = 0
        print("POS ID {}, x(mm): {pos.x} y(mm): {pos.y} z(mm): {pos.z}".format(
            "0x%0.4x" % network_id, pos=position))

    def printPublishErrorCode(self, operation):
        """Prints the Pozyx's error and possibly sends it as a OSC packet"""
        error_code = SingleRegister()
        network_id = self.remote_id
        if network_id is None:
            self.pozyx.getErrorCode(error_code)
            print("LOCAL ERROR %s, %s" % (operation, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, 0, error_code[0]])
            return
        status = self.pozyx.getErrorCode(error_code, self.remote_id)
        if status == POZYX_SUCCESS:
            print("ERROR %s on ID %s, %s" %
                  (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/error", [operation, network_id, error_code[0]])
        else:
            self.pozyx.getErrorCode(error_code)
            print("ERROR %s, couldn't retrieve remote error code, LOCAL ERROR %s" %
                  (operation, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, 0, -1])
            # should only happen when not being able to communicate with a remote Pozyx.

    def setAnchorsManual(self, save_to_flash=False):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        status = self.pozyx.clearDevices(remote_id=self.remote_id)
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, remote_id=self.remote_id)
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(PozyxConstants.ANCHOR_SELECT_AUTO, len(self.anchors),
                                                       remote_id=self.remote_id)

        if save_to_flash:
            self.pozyx.saveAnchorIds(remote_id=self.remote_id)
            self.pozyx.saveRegisters([PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS], remote_id=self.remote_id)
        return status

    def printPublishConfigurationResult(self):
        """Prints and potentially publishes the anchor configuration result in a human-readable way."""
        list_size = SingleRegister()

        self.pozyx.getDeviceListSize(list_size, self.remote_id)
        print("List size: {0}".format(list_size[0]))
        if list_size[0] != len(self.anchors):
            self.printPublishErrorCode("configuration")
            return
        device_list = DeviceList(list_size=list_size[0])
        self.pozyx.getDeviceIds(device_list, self.remote_id)
        print("Calibration result:")
        print("Anchors found: {0}".format(list_size[0]))
        print("Anchor IDs: ", device_list)

        for i in range(list_size[0]):
            anchor_coordinates = Coordinates()
            self.pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates, self.remote_id)
            print("ANCHOR, 0x%0.4x, %s" % (device_list[i], str(anchor_coordinates)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [device_list[i], int(anchor_coordinates.x), int(anchor_coordinates.y), int(anchor_coordinates.z)])
                sleep(0.025)

    def printPublishAnchorConfiguration(self):
        """Prints and potentially publishes the anchor configuration"""
        for anchor in self.anchors:
            print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.coordinates)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [anchor.network_id, int(anchor.coordinates.x), int(anchor.coordinates.y), int(anchor.coordinates.z)])
                sleep(0.025)


if __name__ == "__main__":
    # Check for the latest PyPozyx version. Skip if this takes too long or is not needed by setting to False.
    check_pypozyx_version = True
    if check_pypozyx_version:
        perform_latest_version_check()

    # shortcut to not have to find out the port yourself
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    remote_id = 0x6e66                 # remote device network ID
    remote = False                   # whether to use a remote device
    if not remote:
        remote_id = None

    # enable to send position data through OSC
    use_processing = False

    # configure if you want to route OSC to outside your localhost. Networking knowledge is required.
    ip = "127.0.0.1"
    network_port = 8888

    osc_udp_client = None

    # necessary data for calibration, change the IDs and coordinates yourself according to your measurement
    anchors = [DeviceCoordinates(0x762a, 1, Coordinates(0, 0, 0)),
               DeviceCoordinates(0x6733, 1, Coordinates(3320, -120, 100)),
               DeviceCoordinates(0x671a, 1, Coordinates(4480, 2350, 100)),
               DeviceCoordinates(0x627d, 1, Coordinates(140, 2210, -60))]

    # positioning algorithm to use, other is PozyxConstants.POSITIONING_ALGORITHM_TRACKING
    algorithm = PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY
    # positioning dimension. Others are PozyxConstants.DIMENSION_2D, PozyxConstants.DIMENSION_2_5D
    dimension = PozyxConstants.DIMENSION_2D
    # height of device, required in 2.5D positioning
    height = 1000

    pozyx = PozyxSerial(serial_port)

    pozyx.clearDevices(remote_id)
    if  pozyx.doDiscovery(discovery_type=PozyxConstants.DISCOVERY_ALL_DEVICES, remote_id=remote_id) == POZYX_SUCCESS:
        print("yoink")
        pozyx.printDeviceList(remote_id)
    else:
        print("reeee")

    data = Data([1,2,3,4,5])
    pozyx.getRead(PozyxRegisters.WHO_AM_I, data, remote_id=remote_id)
    print('who am i: 0x%0.2x' % data[0])
    print('firmware version: 0x%0.2x' % data[1])
    print('hardware version: 0x%0.2x' % data[2])
    print('self test result: %s' % bin(data[3]))
    print('error: 0x%0.2x' % data[4])

    r = ReadyToLocalize(pozyx, osc_udp_client, anchors, algorithm, dimension, height, remote_id)
    r.setup()
    while True:
        r.loop()
