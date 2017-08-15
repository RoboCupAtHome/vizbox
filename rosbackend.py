from backendbase import BackendBase, call_callbacks_in


class RosBackend(BackendBase):
    __instance = None

    @staticmethod
    def get_instance(*args, **kwargs):
        if not RosBackend.__instance:
            RosBackend.__instance = RosBackend(*args, **kwargs)
        return RosBackend.__instance

    def __init__(self, shutdown_hook):
        import rospy
        from std_msgs.msg import String, UInt32

        super(RosBackend, self).__init__()
        rospy.init_node("vizbox", log_level=rospy.INFO)
        print "Node initialized"

        rospy.on_shutdown(shutdown_hook)

        self.op_sub = rospy.Subscriber("operator_text", String, call_callbacks_in(self.on_operator_text, lambda rosmsg: rosmsg.data), queue_size=100)
        self.robot_sub = rospy.Subscriber("robot_text", String, call_callbacks_in(self.on_robot_text, lambda rosmsg: rosmsg.data), queue_size=100)
        self.step_sub = rospy.Subscriber("challenge_step", UInt32, call_callbacks_in(self.on_challenge_step, lambda rosmsg: rosmsg.data), queue_size=100)

        self.cmd_pub = rospy.Publisher("command", String, queue_size=1)

    def accept_command(self, command_text):
        self.cmd_pub.publish(command_text)