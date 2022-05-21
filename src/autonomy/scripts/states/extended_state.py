from smach import State
import rospy
from std_msgs.msg import Bool
from abc import ABCMeta, abstractmethod
from actionlib import SimpleGoalState


class ExtendedState(State):
    __metaclass__ = ABCMeta

    def __init__(self, *args, **kwargs):
        State.__init__(self, *args, **kwargs)
        self._autonomy_sub = rospy.Subscriber("set_autonomy", Bool, self.autonomy_callback)
        self._checking_for_change = False
        self._changed = False

    def autonomy_callback(self, msg):
        if self._checking_for_change:
            self._changed = True
            self._checking_for_change = False
    
    def has_autonomy_changed(self):
        if self._changed:
            self._changed = False
            return True
        return False

    def sleep_with_callback(self, duration, callback, rate=10):
        """
        Alias for rospy.sleep, except that this method returns False early if self.spinning = False
        Useful for waiting for some time, but also checking if manual control was taken by remote base
        @param duration: How long to sleep for. Set to 0 to sleep forever
        @param callback: Method to call (without args) on every tick
        @param rate: Rate (Hz) to check for change in autonomy
        @return: True iff autonomy changed
        """
        timer = rospy.Rate(rate)
        if duration == 0:
            while not self.has_autonomy_changed():
                timer.sleep()
                callback()
            return True
        
        for _ in range(int(round(duration * rate))):
            if self.has_autonomy_changed():
                return True
            timer.sleep()
            callback()
        
        return False

    def sleep(self, duration, rate=10):
        """
        Alias for rospy.sleep, except that this method returns False early if self.spinning = False
        Useful for waiting for some time, but also checking if manual control was taken by remote base
        @param duration: How long to sleep for. Set to 0 to sleep forever
        @param rate: Rate (Hz) to check for change in autonomy
        @return: True iff autonomy changed
        """
        timer = rospy.Rate(rate)
        if duration == 0:
            while not self.has_autonomy_changed():
                timer.sleep()
            return True
        
        for _ in range(int(round(duration * rate))):
            if self.has_autonomy_changed():
                return True
            timer.sleep()
        
        return False
    
    def wait_for_action_result(self, action_client, rate=10):
        """
        Waits for the action server to be done, or if manual control was taken
        @param action_client: The action client to use
        @param rate: Rate (Hz) to check for changes
        @return: True iff autonomy changed
        """
        timer = rospy.Rate(rate)
        while not self.has_autonomy_changed():
            if action_client.get_state() == SimpleGoalState.DONE:
                return False
        return True
    
    @abstractmethod
    def execute(self, userdata):
        pass
