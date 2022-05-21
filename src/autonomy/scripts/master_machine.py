#! /usr/bin/env python

import rospy
from smach import StateMachine
from autonomy.msg import StartMachineAction
import actionlib
from states import *


class AutonomyMachine:
    def __init__(self):
        self.mm = StateMachine(outcomes=['finished'])
        self.setGetPoseServer = actionlib.SimpleActionServer(
            "start_machine_as", StartMachineAction, execute_cb=self.startMachine_cb, auto_start=False)

        self.mm.userdata.driving_to_site = True

        with self.mm:
            StateMachine.add('Init', InitializeState(), transitions={'finished': 'InitPosition'})
            StateMachine.add('InitPosition', InitializePositionState(), transitions={'finished': 'Drive', 'manual': 'Manual'})
            
            StateMachine.add('Drive', DriveState(), transitions={'reached_dig_site': 'Dig', 'reached_bin': 'Unload', 'manual': 'Manual'})
            StateMachine.add('Dig', DigState(), transitions={'finished': 'Drive', 'manual': 'Manual'})
            StateMachine.add('Unload', UnloadState(), transitions={'finished': 'Drive', 'manual': 'Manual'})
            StateMachine.add('Manual', ManualState(), transitions={'drive': 'Drive', 'dig': 'Dig', 'unload': 'Unload'})

    def ExecuteMachine(self):
        if not rospy.get_param("/isAutonomous"):
            rospy.logerr("Tried to start autonomy while isAutonomous is false!")
            return
        self.mm.execute()

    def startMachine_cb(self, goal):
        self.ExecuteMachine()


if __name__ == "__main__":
    rospy.init_node("master_machine")
    machine = AutonomyMachine()
    rospy.spin()
