import rospy

import actionlib

import rospy

from std_msgs.msg import Float32
import os

from pyvesc import VESC

# file defining feedback, goal and result
import ActionFile 

class CommandServer(object):
    # create messages that are used to publish feedback/result
     _feedback = actionFile.msg.ActionFileFeedback()
     _result = actionFile.msg.ActionFileResult()

  
     def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, ActionFile.msg.ActionFileAction, 			execute_cb=self.execute_cb, auto_start = False)
	self.drum_motor = VESC(serial_port=serial_port1)
        self.arm_motor = VESC(serial_port=serial_port2)
        self._as.start()


     def execute_cb(self, goal):

        rospy.loginfo('%s:connection success' % self._action_name)
        success = True
        
	# start executing the action
	while(true):
		# check that preempt has not been requested by the client
		if self._as.is_preempt_requested():
                     rospy.loginfo('%s: Preempted' % self._action_name)
                     self._as.set_preempted()
                     success = False
                     break
		
		#Get the goal (command) and do something (lower arm, raise arm, rotate drum in reverse direction)
		if(goal.command == "CMD_ReverseRotate"):
			self.drum_motor.set_duty_cycle(-10)
		if(goal.command == "CMD_RaiseArm"):
			self.arm_motor.set_duty_cycle(10)
		if(goal.command == "CMD_LowerArm):
			self.arm_motor.set_duty_cycle(-10)

		if success:
          		# update self._result = true
             		rospy.loginfo('%s: Succeeded' % self._action_name)
             		self._as.set_succeeded(self._result)
			break
		


if __name__ == '__main__':
      rospy.init_node('command_server')
      server = CommandServer(rospy.get_name())
      rospy.spin()

	

