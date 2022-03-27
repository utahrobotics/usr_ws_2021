#! /usr/bin/env python

import rospy
from smach import StateMachine
from states import *


if __name__ == "__main__":
    rospy.init_node("master_machine")

    # master machine
    mm = StateMachine(outcome=['finished'])

    with mm:
        StateMachine.add('Init', Initialize(), transitions={'finished': 'Drive'})
        StateMachine.add('Drive', Drive(), transitions={'reached_dig_site': 'Dig', 'reached_bin': 'Unload'})
        StateMachine.add('Dig', Dig(), transitions={'finished': 'Drive'})
        StateMachine.add('Unload', Unload(), transitions={'finished': 'Drive'})

    mm.execute()
