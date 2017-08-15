#! /usr/bin/env python

import signal
import rospy

from socket import error

from tornado.ioloop import IOLoop
from tornado.web import Application, RequestHandler, StaticFileHandler
from tornado.websocket import WebSocketHandler

import json

def call_callbacks_in(cb_list, converter):
    def callback(message):
        converted = converter(message)
        for cb in cb_list:
            cb(converted)

    return callback


class BackendBase(object):
    @staticmethod
    def get_instance():
        raise NotImplementedError()

    def __init__(self):
        self.on_operator_text = []
        self.on_robot_text = []
        self.on_challenge_step = []

    def attach_operator_text(self, callback):
        self.on_operator_text += [callback]

    def attach_robot_text(self, callback):
        self.on_robot_text += [callback]

    def attach_challenge_step(self, callback):
        self.on_challenge_step += [callback]

    def detach_operator_text(self, callback):
        self.on_operator_text.remove(callback)

    def detach_robot_text(self, callback):
        self.on_robot_text.remove(callback)

    def detach_challenge_step(self, callback):
        self.on_challenge_step.remove(callback)

    def accept_command(self, command_text):
        raise NotImplementedError()


class RosBackend(BackendBase):
    __instance = None

    @staticmethod
    def get_instance():
        if not RosBackend.__instance:
            RosBackend.__instance = RosBackend()
        return RosBackend.__instance

    def __init__(self):
        import rospy
        from std_msgs.msg import String, UInt32

        super(RosBackend, self).__init__()
        rospy.init_node("vizbox", log_level=rospy.INFO)
        print "Node initialized"

        rospy.on_shutdown(handle_shutdown)

        self.op_sub = rospy.Subscriber("operator_text", String, call_callbacks_in(self.on_operator_text, lambda rosmsg: rosmsg.data), queue_size=100)
        self.robot_sub = rospy.Subscriber("robot_text", String, call_callbacks_in(self.on_robot_text, lambda rosmsg: rosmsg.data), queue_size=100)
        self.step_sub = rospy.Subscriber("challenge_step", UInt32, call_callbacks_in(self.on_challenge_step, lambda rosmsg: rosmsg.data), queue_size=100)

        self.cmd_pub = rospy.Publisher("command", String, queue_size=1)

    def accept_command(self, command_text):
        self.cmd_pub.publish(command_text)


class ChallengeHandler(RequestHandler):
    def get(self):
        print "Rendering..."
        self.render("challenge.html",
                    challenge="Help me Carry",
                    story=["Get operator", "Follow to car", "Take bag", "Hear destination", "Find human", "Guide to car"],
                    robot_text="OK, I will bring the bag to the kitchen",
                    operator_text="Bring the bag to the kitchen",
                    visualization="Insert robot doing awesome stuff"
                    )


class CommandReceiver(RequestHandler):
    def initialize(self, backend):
        self.backend = backend

    def post(self, *args, **kwargs):
        command = self.get_argument("command")
        self.backend.accept_command(command)
        print(command)


class MessageForwarder(WebSocketHandler):

    def __init__(self, *args, **kwargs):
        self.backend = kwargs.pop('backend')
        super(MessageForwarder, self).__init__(*args, **kwargs)

    def check_origin(self, origin):
        return True

    def open(self):
        self.backend.attach_operator_text(self.handle_operator_text)
        self.backend.attach_robot_text(self.handle_robot_text)
        self.backend.attach_challenge_step(self.handle_challenge_step)

        print("WebSocket opened")

    def on_message(self, message):
        self.write_message(u"You said: " + message)

    def on_close(self):
        print("WebSocket closed")
        self.backend.detach_operator_text(self.handle_operator_text)
        self.backend.detach_robot_text(self.handle_robot_text)
        self.backend.detach_challenge_step(self.handle_challenge_step)

    def handle_operator_text(self, text):
        print "handle_operator_text({})".format(text)

        data = {"label": "operator_text", "text": text}
        data = json.dumps(data)

        self.write_message(data)

    def handle_robot_text(self, text):
        print "handle_robot_text({})".format(text)

        data = {"label": "robot_text", "text": text}
        data = json.dumps(data)

        self.write_message(data)

    def handle_challenge_step(self, step):
        print "handle_challenge_step({})".format(step)

        data = {"label": "challenge_step", "index": step}
        data = json.dumps(data)

        self.write_message(data)


def handle_shutdown(*arg, **kwargs):
    IOLoop.instance().stop()

if __name__ == "__main__":
    backend = RosBackend.get_instance()

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGQUIT, handle_shutdown) # SIGQUIT is send by our supervisord to stop this server.
    signal.signal(signal.SIGTERM, handle_shutdown) # SIGTERM is send by Ctrl+C or supervisord's default.
    print "Shutdown handler connected"

    app = Application([
        (r"/ws", MessageForwarder, {'backend': backend}),
        (r'/', ChallengeHandler),
        (r'/command', CommandReceiver, {'backend': backend}),
        (r'/static/(.*)', StaticFileHandler, {'path': 'static/'})],
        (r'/(favicon\.ico)', StaticFileHandler, {'path': 'static/favicon.ico'}),
    debug=True,
    template_path="templates")

    address, port = "localhost", 8888
    print "Application instantiated"

    connected = False
    while not connected and not rospy.is_shutdown():
        try:
            print "Listening..."
            app.listen(port, address)
            rospy.logdebug("Listening on {addr}:{port}".format(addr=address, port=port))
            connected = True
        except error as ex:
            rospy.logerr("{ex}. Cannot start, trying in a bit".format(ex=ex))
            rospy.sleep(1)

    print "Starting IOLoop"
    IOLoop.instance().start()