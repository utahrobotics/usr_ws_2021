import rospy
from abstract_server import AbstractActionServer
from autonomy.msg import MoveArmAction, MoveDiggerAction
from std_msgs.msg import Float32
from math import pi


class MoveArmServer(AbstractActionServer):
    def __init__(self):
        AbstractActionServer.__init__(self, 'move_arm', MoveArmAction)
	    self.arm_vel_pub = rospy.Publisher('arm_vel', Float32, queue_size=1)
	    self.arm_angle_sub = rospy.Subscriber('/sensors/angleSensor/angle', Float32, self.set_arm_angle, queue_size=1)
	    self.arm_angle = 0.0

    def set_arm_angle(self, angle):
        self.arm_angle = angle

    def execute(self, goal):
        r = rospy.Rate(20)
        if goal.extend:
            self.arm_vel_pub.publish(Float32(1))        # 1 radian per second?
            while self.arm_angle < pi / 6:
                r.sleep()
		        self.publish_feedback(self.arm_angle / (pi / 6))
            self.arm_vel_pub.publish(Float32(0))
        else:
            self.arm_vel_pub.publish(Float32(-1))        # 1 radian per second?
            while self.arm_angle > 0:
                r.sleep()
		        self.publish_feedback(1 - self.arm_angle / (pi / 6))
            self.arm_vel_pub.publish(Float32(0))

        if goal.extend:
            rospy.logwarn("Extended Arm")
        else:
            rospy.logwarn("Retracted Arm")

        return 0


class MoveDiggerServer(AbstractActionServer):
    def __init__(self):
	    self.drum_vel_pub = rospy.Publisher('drum_vel', Float32, queue_size=1)
        AbstractActionServer.__init__(self, 'move_digger', MoveDiggerAction)

    def execute(self, goal):
	    if goal.digging:
	        self.drum_vel_pub.publish(Float32(1))		# 1 radian per second?
    	else:
	        self.drum_vel_pub.publish(Float32(-1))		# reverse 1 radian per second?

        r = rospy.Rate(10)
        count = int(round(goal.duration * 10))
	    for i in range(count):
            r.sleep()
            self.publish_feedback(i / count)

	    self.drum_vel_pub.publish(Float32(0))
        if goal.digging:
            rospy.logwarn("Dug")
        else:
            rospy.logwarn("Unloaded")

        return 0
