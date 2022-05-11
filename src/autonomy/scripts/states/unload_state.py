import rospy
from extended_state import ExtendedState


class UnloadState(ExtendedState):
    def __init__(self):
        ExtendedState.__init__(
            self,
            outcomes=['finished', 'manual'],
            input_keys=["action_client"],
            output_keys=["action_client"]
        )

    def execute(self, userdata):
        userdata.current_state = 'Unload'
        rospy.logwarn('Extending Arm')
        userdata.action_client.move_arm(True).wait_for_result()
        rospy.logwarn('Unloading')
        userdata.action_client.move_digger(False).wait_for_result()
        rospy.logwarn('Retracting Arm')
        userdata.action_client.move_arm(False).wait_for_result()
        rospy.logwarn('Arm Retracted')
        return 'finished'
