from smach import State


class Unload(State):
    def __init__(self):
        State.__init__(self, outcomes=['finished'])

    def execute(self, userdata):
        rospy.loginfo('Unloading complete')
        return 'finished'
