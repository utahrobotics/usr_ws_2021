import rospy
from extended_state import ExtendedState


class DigState(ExtendedState):
    def __init__(self):
        ExtendedState.__init__(
            self,
            outcomes=['finished', 'manual'],
            input_keys=["driving_to_site", "action_client"],
            output_keys=["driving_to_site", "action_client"]
        )

    def execute(self, userdata):
        userdata.current_state = 'Dig'
        userdata.driving_to_site = False
        rospy.logwarn('Extending Arm')
        userdata.action_client.move_arm(True).wait_for_result()
        rospy.logwarn('Digging')
        userdata.action_client.move_digger(True).wait_for_result()
        rospy.logwarn('Retracting Arm')
        userdata.action_client.move_arm(False).wait_for_result()
        rospy.logwarn('Arm Retracted')
        return 'finished'
