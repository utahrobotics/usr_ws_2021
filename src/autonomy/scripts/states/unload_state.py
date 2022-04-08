from smach import State
import rospy


class UnloadState(State):
    def __init__(self):
        State.__init__(self, outcomes=['finished'])

    def execute(self, userdata):
        rospy.logwarn('Unloading complete')
        return 'finished'
