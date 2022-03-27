from smach import State


class Dig(State):
    def __init__(self):
        State.__init__(self, outcomes=['finished'])

    def execute(self, userdata):
        rospy.loginfo('Dig complete')
        return 'finished'
