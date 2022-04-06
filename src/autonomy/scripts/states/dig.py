from smach import State
import rospy


class Dig(State):
    def __init__(self):
        State.__init__(self, outcomes=['finished'], input_keys=["driving_to_site"], output_keys=["driving_to_site"])

    def execute(self, userdata):
        userdata.driving_to_site = False
        rospy.logwarn('Dig complete')
        return 'finished'
