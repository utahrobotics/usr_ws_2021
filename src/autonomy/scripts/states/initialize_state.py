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
        rospy.logwarn("Fake initing")
        goal = FakeInitGoal()
        goal.goal = True
        self.fake_init_client.send_goal(goal)
        if self.wait_for_action_result(self.fake_init_client):
            return 'manual'
        rospy.logwarn('Fake initialized')
        return 'finished'
