from smach import State
import rospy
from std_msgs.msg import Bool
from abc import ABCMeta


class ExtendedState(State):
    __metaclass__ = ABCMeta

    def __init__(self, *args, **kwargs):
        State.__init__(self, *args, **kwargs)
        self.spinning = False
        self._autonomy_sub = rospy.Subscriber("set_autonomy", Bool, self.callback)

    def callback(self, msg):
        if msg.data:
            self.spinning = False

    def sleep(self, duration, rate=10):
        """
        Alias for rospy.sleep, except that this method returns False early if self.spinning = False
        Useful for waiting for some time, but also checking if manual control was taken by remote base
        @param duration: How long to sleep for
        @param rate: Rate (Hz) to check for change in autonomy
        @return: True iff still spinning
        """
        timer = rospy.Rate(rate)
        for _ in range(int(round(duration * rate))):
            if not self.spinning:
                return False
            timer.sleep()
        return self.spinning
