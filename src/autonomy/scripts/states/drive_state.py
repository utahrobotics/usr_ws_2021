import rospy
from extended_state import ExtendedState


class DriveState(ExtendedState):
    def __init__(self):
        ExtendedState.__init__(
            self,
            outcomes=['reached_dig_site', 'reached_bin', 'manual'],
            input_keys=["driving_to_site", "action_client"],
            output_keys=["action_client"]
        )

    def execute(self, userdata):
        userdata.current_state = 'Drive'
        if userdata.driving_to_site:
            # userdata.action_client.move_to(5, 2).wait_for_result()
            userdata.action_client.move_to(5, 2)
            rospy.sleep(1)
            rospy.logwarn('Drove to dig site')
            return 'reached_dig_site'
        # userdata.action_client.move_to(0, 2).wait_for_result()
        userdata.action_client.move_to(0, 2)
        rospy.sleep(1)
        rospy.logwarn('Drove to bin')
        return 'reached_bin'
