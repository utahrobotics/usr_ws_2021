from smach import State


class Drive(State):
    def __init__(self):
        State.__init__(self, outcomes=['reached_dig_site', 'reached_bin'])

    def execute(self, userdata):
        rospy.loginfo('Drive complete')
        return 'finished'
