import rospy
from extended_state import ExtendedState
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class DriveState(ExtendedState):
	def __init__(self):
		self.move_clients = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.move_clients.wait_for_server(rospy.Duration(3))
		ExtendedState.__init__(
			self,
			outcomes=['reached_dig_site', 'reached_bin', 'manual'],
			input_keys=["driving_to_site"],
		)
	
	def _move_to(self, x, y):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y
		# goal.target_pose.pose.orientation.w = 1.0
		self.move_clients.send_goal(goal)
		return self.wait_for_action_result(self.move_clients)
	
	def execute(self, userdata):
		userdata.current_state = 'Drive'
		if userdata.driving_to_site:
			if self._move_to(1, 5.64):
				return 'manual'
		elif self._move_to(0, 1.1):
			return 'manual'
		rospy.logwarn('Drove to bin')
		return 'reached_bin'
