import signal
import rospy

from socket import error

from tornado.ioloop import IOLoop
from tornado.web import Application, RequestHandler, StaticFileHandler
from tornado.websocket import WebSocketHandler

from std_msgs.msg import String

import json

ws_clients = []

class ChallengeHandler(RequestHandler):
    def get(self):
        print "Rendering..."
        self.render("challenge.html",
                    challenge="Help me Carry",
                    story=["Follow to car", "Take bag", "Guide to car"],
                    robot_text="OK, I will bring the bag to the kitchen",
                    operator_text="Bring the bag to the kitchen",
                    visualization="Insert robot doing awesome stuff"
                    )

class RosMessageForwarder(WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        # TODO: Only subscribe once for the whole application
        self.sub1 = rospy.Subscriber("/operator_text", String, self.handle_operator_text, queue_size=100)
        self.sub2 = rospy.Subscriber("/robot_text", String, self.handle_robot_text, queue_size=100)

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

    def handle_operator_text(self, rosmsg):
        print "handle_operator_text({})".format(rosmsg)

        data = {"label": "operator_text", "text": rosmsg.data}
        data = json.dumps(data)

        for c in ws_clients:
            c.write_message(data)

    def handle_robot_text(self, rosmsg):
        print "handle_robot_text({})".format(rosmsg)

        data = {"label": "robot_text", "text": rosmsg.data}
        data = json.dumps(data)

        for c in ws_clients:
            c.write_message(data)

def handle_shutdown(*arg, **kwargs):
    IOLoop.instance().stop()

if __name__ == "__main__":
    rospy.init_node("vizbox", log_level=rospy.INFO)
    print "Node initialized"

    rospy.on_shutdown(handle_shutdown)
    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGQUIT, handle_shutdown) # SIGQUIT is send by our supervisord to stop this server.
    signal.signal(signal.SIGTERM, handle_shutdown) # SIGTERM is send by Ctrl+C or supervisord's default.
    print "Shutdown handler connected"

    app = Application([
        (r"/ws", RosMessageForwarder),
        (r'/', ChallengeHandler),
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