from backendbase import BackendBase, call_callbacks_in

import rospy
from std_msgs.msg import String, UInt32
from sensor_msgs.msg import Image

from PIL import Image as pil_image
import base64
from StringIO import StringIO

HEADER = "data:image/png;base64,"
DUMMY1 = "iVBORw0KGgoAAAANSUhEUgAAADIAAAAyCAMAAAAp4XiDAAAASFBMVEUAAAAAAADMzMz/AABmZmaZmZnMAABmAACZAAAzAAAzMzP/MzPMZmaZMzPMMzP/MwD/////Zmb/ZgBmMzPMmZmZZmb/mQD/mZlxvxKlAAAAAXRSTlMAQObYZgAAAAFiS0dEAIgFHUgAAAAJcEhZcwAACxMAAAsTAQCanBgAAAAHdElNRQffCBIMMyVTqZicAAACK0lEQVRIx43W63aDIAwA4KJJhVjrVtfu/d90kASIFlzzp5fDd5KAgJdLJwa8fB6YYojBXz4EQw1EgNPxAGDHqzpDBqDV0EN5DDoToIhagkSAOwb2DIEBRLEWINJf2DQspCQ7AwiUy0M69pEFyCR/xZCl4cTE/CgQtYoU2xZC2J6YiXNHA3ai4uApRwjaj+RBs+JWpKHLcrstiyAz3WjFUFJMk4/jf39+fqPyCdXJLmmwzi4L7+8a8avPhlwtzSQRcY0x3+9z+mSz8bSY0jRJWoEirtdxlM+IluczEXA7In+FbVm8jJ9jqIpmC2nBSjc6XXFZwvS83XwC4/wdI5pRDLczUE6TSJoPisRrDhNCkqFSmTY/pN6ZpIo05pguGZk1qawSKL3PxnznNIGHkDSjK1/q4tZNnlFI0CdNCeiTooRjXVfTTV5PJnggWTxejzNCDfJ6PdYWGd6zXHOWR7ewXi/jOrYJ8PbCvFH8cSGFlBmjQoZC/Dup68LbjAmWR39fWaOu1MRFCNg0rWesrCQTU1njuaxPZW1lX1neYRXwvgy1rkLQGN2WZVOWJHFPDZBPMUTXMVZAvZ+gnolqfBlvT6VEymGJelzryedL7M4+MFdTvIwAnDn8JnPCupbgO7KaeijHLz1xNKzseBVweTPketEQQgDagLAhssFWIl6P5j3OpK5QzgB8lXVufiFoFkl/n71fZIPlDeYsxa64+l7yH1BFUAv6ANgZ7wz/A295IO9de7kCAAAAAElFTkSuQmCC"
DUMMY2 = "iVBORw0KGgoAAAANSUhEUgAAADIAAAAyBAMAAADsEZWCAAAAFVBMVEVvcm0AAAAzMzNmZmbMzMyZmZn///8pcdebAAAAAXRSTlMAQObYZgAAAAFiS0dEAIgFHUgAAAAJcEhZcwAACxMAAAsTAQCanBgAAAAHdElNRQffCBINBR5qfIr6AAABv0lEQVQ4y22UQZaCMAyGffo8QED3Q9E9EDmAYzrrgZIbzJv7H2GStFLLmA22H/+fWNLsdi/hd2+DeSHP/A6Qhg9duX/AhVJskAGPoyLsNmACjVrQkMFe3u0hxUKPj5W0BqoWh0YeSxbtb2o1xvyNGPqnCMkLYPr6/SEOABcKqeKbF8mV2UE1KgHysbyDStaQTCKKR8GpYHDoIJVuJ3hcyJZVK1ZanFbuv5XQIypaRBycoupkdkx3kzhBg3NOMl2j3UJ9LS+6NRCufSLyq9KtqBFVPWmi4+KBoUbZkQoUiMHniZTcYYYqmylpjUgBdyVohu2s5HxOpDdiSWbNA3UlZR/lBBqw/CYyIkenRL9ZqjfVFgln0vJckiaTkIkvNYXba57WZRLku03xPHPo2sj9DamVLFuiFZ2pE2IfbkNOQnY32hBdXrR7RoLSzhrhYWQqRNYj/NB281MhsrZiVGJtuYpiw7E16RiajGIrYrxdGHooo0ZMdxQ3BJ8XErHfSrrnxcamlOB65Qu/K+bbLWTOXrxKDKU/JU3I/DoQDsjMgwmYyyFyGNkGj4ykgOXgET8bSf+AqAZ1DIhvBhwivu7/ATYxlvOTH50RAAAAAElFTkSuQmCC"

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
        self.image_sub = rospy.Subscriber("image", Image, call_callbacks_in(self.on_image, self.ros_image_to_base64), queue_size=100)

        self.cmd_pub = rospy.Publisher("command", String, queue_size=1)

    def accept_command(self, command_text):
        self.cmd_pub.publish(command_text)

    def ros_image_to_base64(self, rosmsg):
        import random
        # return random.choice([DUMMY1, DUMMY2])
        print rosmsg.encoding, rosmsg.width, rosmsg.height, len(rosmsg.data)

        return self.__encoding[rosmsg.encoding](rosmsg)

    @staticmethod
    def rgba2base64(rosmsg):
        length = len(rosmsg.data)
        bytes_needed = int(rosmsg.width * rosmsg.height * 3)
        # print "encode: length={} width={}, heigth={}, bytes_needed={}".format(length, width, height, bytes_needed)

        converted = pil_image.frombytes('RGB',
                                        (rosmsg.width, rosmsg.height),
                                        rosmsg.data)
        string_buffer = StringIO()
        converted.save(string_buffer, "png")
        image_bytes = string_buffer.getvalue()
        encoded = base64.standard_b64encode(image_bytes)
        return encoded