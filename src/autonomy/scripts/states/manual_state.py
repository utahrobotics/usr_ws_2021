import rospy
from extended_state import ExtendedState


class ManualState(ExtendedState):
    def __init__(self):
        ExtendedState.__init__(
            self,
            outcomes=['drive', 'dig', 'unload'],
            input_keys=["action_client", 'current_state'],
            output_keys=["action_client", 'current_state']
        )

    def callback(self, msg):
        if msg.data: return
        self.spinning = False
        rospy.logwarn("EXITING MANUAL MODE")

    def execute(self, userdata):
        rospy.logwarn("ENTERING MANUAL MODE")
        self.spinning = True
        timer = rospy.Rate(10)      # 10 Hz

        while self.spinning:
            timer.sleep()

        if userdata.current_state == "Drive":
            return 'drive'
        elif userdata.current_state == "Dig":
            return 'dig'
        elif userdata.current_state == "Unload":
            return 'unload'
        else:
            raise KeyError("Unrecognized state: " + userdata.current_state)
