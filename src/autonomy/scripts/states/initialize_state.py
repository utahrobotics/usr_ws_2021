import rospy
from extended_state import ExtendedState
from motors.msg import FakeInitAction, FakeInitGoal
from actionlib import SimpleActionClient


class InitializeState(ExtendedState):
    def __init__(self):
        self.fake_init_client = SimpleActionClient("fake_init_as", FakeInitAction)
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
        goal = FakeInitGoal()
        goal.goal = True
        self.fake_init_client.send_goal(goal)
        #TODO: rather than just waiting, be aware of making robot manual and switch to appropriate state.
        self.fake_init_client.wait_for_result()
        rospy.logwarn('Initialized')
        return 'finished'
