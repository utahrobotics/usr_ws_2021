from actionlib import SimpleActionServer
from localization.msgs import StartTripOdometerAction, StopTripOdometerAction, StartTripOdometerFeedback
import tf
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import rospy


SPINNING_TRIP_ODOM = False


class StartTripOdometer(object):
	def __init__(self):
		self.action_server = SimpleActionServer("start_trip_odometer", StartTripOdometerAction, self.execute)
		self.tf_listener = tf.TransformListener()
		self.action_server.start()
	
	def execute(self):
		global SPINNING_TRIP_ODOM
		SPINNING_TRIP_ODOM = True
		
		while SPINNING_TRIP_ODOM:
			feedback = StartTripOdometerFeedback()
			origin, rotation = self.tf_listener.lookupTransform("/map", "/base_link", rospy.Time(0))
			feedback.pose = Pose()
			feedback.pose.position = origin
			feedback.pose.orientation = quaternion_from_euler(rotation)
			self.action_server.publish_feedback()


class StopTripOdometer(object):
	def __init__(self):
		self.action_server = SimpleActionServer("stop_trip_odometer", StopTripOdometerAction, self.execute)
		self.action_server.start()
	
	def execute(self):
		global SPINNING_TRIP_ODOM
		SPINNING_TRIP_ODOM = False


if __name__ == "__main__":
	rospy.init_node("trip_odometer")
	start= StartTripOdometer()
	stop = StopTripOdometer()
	rospy.spin()
