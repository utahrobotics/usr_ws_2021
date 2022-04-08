#! /usr/bin/env python

import rospy
from smach import StateMachine
from states import *


if __name__ == "__main__":
    rospy.init_node("master_machine")

    # master machine
    mm = StateMachine(outcomes=['finished'])
    mm.userdata.driving_to_site = True

    action_client = ActionClient()
    action_client.wait_for_servers()
    mm.userdata.action_client = action_client

    with mm:
        StateMachine.add('Init', InitializeState(), transitions={'finished': 'Drive'})
        StateMachine.add('Drive', DriveState(), transitions={'reached_dig_site': 'Dig', 'reached_bin': 'Unload'})
        StateMachine.add('Dig', DigState(), transitions={'finished': 'Drive'})
        StateMachine.add('Unload', UnloadState(), transitions={'finished': 'finished'})

    mm.execute()
    rospy.logwarn("State machine finished!")
