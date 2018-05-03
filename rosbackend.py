from backendbase import BackendBase, call_callbacks_in

import rospy
from std_msgs.msg import String, UInt32
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np
from PIL import Image as pil_image
import base64
from StringIO import StringIO

try:
    from vizbox.msg import Story
except ImportError as e:
    rospy.logerr(e)


class RosBackend(BackendBase):
    __instance = None


    @staticmethod
    def get_instance(*args, **kwargs):
        if not RosBackend.__instance:
            RosBackend.__instance = RosBackend(*args, **kwargs)
        return RosBackend.__instance

    def __init__(self, shutdown_hook):

        super(RosBackend, self).__init__()
        rospy.init_node("vizbox", log_level=rospy.INFO)
        print "Node initialized"

        rospy.on_shutdown(shutdown_hook)

        self.__encoding = {'rgb8':self.rgba2base64}

        self.op_sub = rospy.Subscriber("operator_text", String, call_callbacks_in(self.on_operator_text, lambda rosmsg: rosmsg.data), queue_size=100)
        self.robot_sub = rospy.Subscriber("robot_text", String, call_callbacks_in(self.on_robot_text, lambda rosmsg: rosmsg.data), queue_size=100)
        self.step_sub = rospy.Subscriber("challenge_step", UInt32, call_callbacks_in(self.on_challenge_step, lambda rosmsg: rosmsg.data), queue_size=100)
        self.image_sub = rospy.Subscriber("image", CompressedImage, call_callbacks_in(self.on_image, self.ros_image_to_base64), queue_size=100)
        
        try:
            self.story_sub = rospy.Subscriber("story", Story, call_callbacks_in(self.on_story, lambda rosmsg: (rosmsg.title, rosmsg.storyline)), queue_size=100)
        except NameError, e:
            rospy.logerr("To dynamically define a Story, catkin_make this package")

        self.cmd_pub = rospy.Publisher("command", String, queue_size=1)

    def accept_command(self, command_text):
        self.cmd_pub.publish(command_text)

    def ros_image_to_base64(self, rosmsg):
        print rosmsg.format, len(rosmsg.data)
        return self.__encoding['rgb8'](rosmsg)  # forcing encoding option at the moment

    @staticmethod
    def rgba2base64(rosmsg):
        length = len(rosmsg.data)
        img_np_arr = np.fromstring(rosmsg.data, np.uint8)
        flag = cv2.IMREAD_COLOR if cv2.__version__.split('.')[0] == '3' else cv2.CV_LOAD_IMAGE_COLOR
        encoded_img = cv2.imdecode(img_np_arr, flag)
        converted = pil_image.fromarray(encoded_img)
        string_buffer = StringIO()
        converted.save(string_buffer, "png")
        image_bytes = string_buffer.getvalue()
        encoded = base64.standard_b64encode(image_bytes)
        return encoded
