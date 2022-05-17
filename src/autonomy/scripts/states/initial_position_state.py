import rospy
from extended_state import ExtendedState
from localization.msgs import GetPoseAction, GetPoseGoal, GetPoseFeedback, GetPoseResult
from actionlib import SimpleActionClient


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
        userdata.current_state = 'Init'
        rospy.logwarn("Initializing")
        #userdata.action_client.initialize().wait_for_result()
        goal = GetPoseGoal()
        #TODO: average result of 10 or so measurements for initial position
        self.get_pose_client.send_goal(goal)
        #TODO: rather than just waiting, be aware of making robot manual and switch to appropriate state.
        self.get_pose_client.wait_for_result()
        initial_pose_est = self.get_pose_client.get_result().pose
        rospy.logwarn("Initial Pose Estimate: x: "+str(initial_pose_est.pose.pose.position.x)+" y: "+str(initial_pose_est.pose.pose.position.y)+" z: "+str(initial_pose_est.pose.pose.position.z))
        #TODO: use fiducials to obtain initial orientation and improve initial posiiton
        #TODO: publish the map -> odom tranform based on the initial pose
        rospy.logwarn('Inintial Pose Obtained')
        return 'finished'
