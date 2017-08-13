import signal
import rospy

from socket import error

from tornado.ioloop import IOLoop
from tornado.web import Application, RequestHandler, StaticFileHandler
from tornado.websocket import WebSocketHandler

from std_msgs.msg import String, UInt32

import json

ws_clients = []

class RosBackend(object):
    __instance = None

    @staticmethod
    def get_instance():
        if not RosBackend.__instance:
            RosBackend.__instance = RosBackend()
        return RosBackend.__instance

    def __init__(self):
        rospy.init_node("vizbox", log_level=rospy.INFO)
        print "Node initialized"

        rospy.on_shutdown(handle_shutdown)

        self.op_sub = rospy.Subscriber("operator_text", String, self.handle_operator_text, queue_size=100)
        self.robot_sub = rospy.Subscriber("robot_text", String, self.handle_robot_text, queue_size=100)
        self.step_sub = rospy.Subscriber("challenge_step", UInt32, self.handle_challenge_step, queue_size=100)

        self.cmd_pub = rospy.Publisher("command", String, queue_size=1)

        self.on_operator_text = []
        self.on_robot_text = []
        self.on_challenge_step = []

    def handle_operator_text(self, rosmsg):
        for handler in self.on_operator_text:
            handler(rosmsg.data)

    def handle_robot_text(self, rosmsg):
        for handler in self.on_robot_text:
            handler(rosmsg.data)

    def handle_challenge_step(self, rosmsg):
        for handler in self.on_challenge_step:
            handler(rosmsg.data)

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
    def post(self, *args, **kwargs):
        command = self.get_argument("command")
        RosBackend.get_instance().cmd_pub.publish(command)
        print(command)


class RosMessageForwarder(WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        RosBackend.get_instance().on_operator_text += [self.handle_operator_text]
        RosBackend.get_instance().on_robot_text += [self.handle_robot_text]
        RosBackend.get_instance().on_challenge_step += [self.handle_challenge_step]

        if self not in ws_clients:
            ws_clients.append(self)
        print("WebSocket opened. {} clients".format(len(ws_clients)))

    def on_message(self, message):
        self.write_message(u"You said: " + message)

    def on_close(self):
        print("WebSocket closed")
        if self in ws_clients:
            ws_clients.remove(self)
        print("{} clients remaining".format(len(ws_clients)))

        RosBackend.get_instance().on_operator_text -= [self.handle_operator_text]
        RosBackend.get_instance().on_robot_text -= [self.handle_robot_text]
        RosBackend.get_instance().on_challenge_step -= [self.handle_challenge_step]

    def handle_operator_text(self, text):
        print "handle_operator_text({})".format(text)

        data = {"label": "operator_text", "text": text}
        data = json.dumps(data)

        for c in ws_clients:
            c.write_message(data)

    def handle_robot_text(self, text):
        print "handle_robot_text({})".format(text)

        data = {"label": "robot_text", "text": text}
        data = json.dumps(data)

        for c in ws_clients:
            c.write_message(data)

    def handle_challenge_step(self, step):
        print "handle_challenge_step({})".format(step)

        data = {"label": "challenge_step", "index": step}
        data = json.dumps(data)

        for c in ws_clients:
            c.write_message(data)

def handle_shutdown(*arg, **kwargs):
    IOLoop.instance().stop()

if __name__ == "__main__":
    RosBackend.get_instance()

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGQUIT, handle_shutdown) # SIGQUIT is send by our supervisord to stop this server.
    signal.signal(signal.SIGTERM, handle_shutdown) # SIGTERM is send by Ctrl+C or supervisord's default.
    print "Shutdown handler connected"

    app = Application([
        (r"/ws", RosMessageForwarder),
        (r'/', ChallengeHandler),
        (r'/command', CommandReceiver),
        (r'/static/(.*)', StaticFileHandler, {'path': 'static/'})],
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