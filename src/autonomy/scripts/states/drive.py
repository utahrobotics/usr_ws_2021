from smach import State
import rospy


class Drive(State):
    def __init__(self):
        State.__init__(self, outcomes=['reached_dig_site', 'reached_bin'], input_keys=["driving_to_site"])

    def execute(self, userdata):
        rospy.loginfo('Drive complete')

        if userdata.driving_to_site:
            return 'reached_dig_site'
        return 'reached_bin'
