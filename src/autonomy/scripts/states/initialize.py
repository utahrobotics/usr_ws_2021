from smach import State


class Initialize(State):
    def __init__(self):
        State.__init__(self, outcomes=['finished'])

    def execute(self, userdata):
        rospy.loginfo('Initialized')
        return 'initialized'
