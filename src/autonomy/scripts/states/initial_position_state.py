import rospy
from extended_state import ExtendedState
from localization.msgs import GetPoseAction, GetPoseGoal, GetPoseFeedback, GetPoseResult
from actionlib import SimpleActionClient
import numpy as np


POSITION_AVERAGE_COUNT = 10


class InitializePositionState(ExtendedState):
	def __init__(self):
		self.get_pose_client = SimpleActionClient("get_pose_as", GetPoseAction)
		ExtendedState.__init__(
			self,
			outcomes=['finished', 'manual'],
			input_keys=["action_client"],
			output_keys=["action_client"]
		)
	
	def execute(self, userdata):
		rospy.logwarn("Initializing position")
		position_sum = np.asarray([0.0, 0.0, 0.0])
		
		for _ in range(POSITION_AVERAGE_COUNT):
			goal = GetPoseGoal()
			self.get_pose_client.send_goal(goal)
			self.wait_for_action_result(self.get_pose_client)
			pose_est = self.get_pose_client.get_result().pose
			position_sum += np.asarray([
				pose_est.pose.pose.position.x,
				pose_est.pose.pose.position.y,
				pose_est.pose.pose.position.z
			])
		
		position_sum /= POSITION_AVERAGE_COUNT
		
		rospy.logwarn(
			"Initial Pose Estimate: x: "+str(position_sum[0])+" y: "+str(position_sum[1])+" z: "+str(position_sum[2])
		)
		# TODO: use fiducials to obtain initial orientation and improve initial posiiton
		# TODO: publish the map -> odom transform based on the initial pose
		rospy.logwarn('Initial Pose Obtained')
		
		# We don't want to go manual during the initialization process
		# But once we are done, we can check if remote base was asking to turn autonomous
		if rospy.get_param("/isAutonomous"):
			return "manual"
		
		return 'finished'
