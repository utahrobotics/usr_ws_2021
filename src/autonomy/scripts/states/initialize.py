from smach import State
import rospy


class Initialize(State):
    def __init__(self):
        State.__init__(self, outcomes=['finished'], input_keys=["action_client"])

    def execute(self, userdata):
        userdata.action_client.initialize().wait_for_result()
        rospy.loginfo('Initialized')
        return 'finished'
