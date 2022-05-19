#! /usr/bin/env python

import rospy
from smach import StateMachine
from autonomy.msg import StartMachineAction, StartMachineGoal, StartMachineFeedback, StartMachineResult
import actionlib
from states import *


class AutonomyMachine:
    def __init__(self):
        self.mm = StateMachine(outcomes=['finished'])
        self.setGetPoseServer = actionlib.SimpleActionServer(
            "start_machine_as", StartMachineAction, execute_cb=self.startMachine_cb, auto_start=False)

    def InitializeMachine(self):
        self.mm.userdata.driving_to_site = True

        action_client = ActionClient()
        action_client.wait_for_servers()
        self.mm.userdata.action_client = action_client

        with self.mm:
            StateMachine.add('Init', InitializeState(), transitions={'finished': 'Drive'})
            StateMachine.add('Initial Position', InitializePositionState(), transitions={'finished':'Drive', 'manual':'Manual'})
            StateMachine.add('Drive', DriveState(), transitions={'reached_dig_site': 'Dig', 'reached_bin': 'Unload', 'manual': 'Manual'})
            StateMachine.add('Dig', DigState(), transitions={'finished': 'Drive', 'manual': 'Manual'})
            StateMachine.add('Unload', UnloadState(), transitions={'finished': 'finished', 'manual': 'Manual'})
            StateMachine.add('Manual', ManualState(), transitions={'drive': 'Drive', 'dig': 'Dig', 'unload': 'Unload'})

    def ExecuteMachine(self):
        self.mm.execute()

    def startMachine_cb(self, goal):
        self.ExecuteMachine()


if __name__ == "__main__":
    rospy.init_node("master_machine")

    machine = AutonomyMachine()
    machine.InitializeMachine()
    machine.ExecuteMachine()
    rospy.logwarn("State machine finished!")
