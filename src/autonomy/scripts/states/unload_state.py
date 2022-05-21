import rospy
from extended_state import ExtendedState
from actionlib import SimpleActionClient
from autonomy.msgs import DumpAction, DumpGoal


class UnloadState(ExtendedState):
    def __init__(self):
        self.dump_server = SimpleActionClient("Dump", DumpAction)
        ExtendedState.__init__(
            self,
            outcomes=['finished', 'manual'],
            input_keys=["action_client"],
            output_keys=["action_client"]
        )

    def execute(self, userdata):
        userdata.current_state = 'Unload'
        self.dump_server.send_goal(DumpGoal())
        self.wait_for_action_result(self.dump_server)
        return 'finished'
