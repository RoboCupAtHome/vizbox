#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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
        self.render("challenge_tornado.html",
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
        self.sub = rospy.Subscriber("/operator_text", String, self.handle_rosmsg, queue_size=100)

        if self not in ws_clients:
            ws_clients.append(self)
        print("WebSocket opened")

    def on_message(self, message):
        self.write_message(u"You said: " + message)

    def on_close(self):
        print("WebSocket closed")
        if self in ws_clients:
            ws_clients.remove(self)

    def handle_rosmsg(self, rosmsg):
        print "handle_rosmsg({})".format(rosmsg)

        data = {"id": 1, "value": rosmsg.data}
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