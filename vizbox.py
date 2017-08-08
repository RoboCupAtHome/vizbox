#! /usr/bin/env python

from flask import Flask
from flask import render_template
from flask_socketio import SocketIO, send, emit

app = Flask(__name__)

socketio = SocketIO(app)

@app.route('/')
def challenge():
    return render_template('challenge.html', 
        challenge="Help me Carry",
        story=["Follow to car", "Take bag", "Guide to car"],
        robot_text="OK, I will bring the bag to the kitchen",
        operator_text="Bring the bag to the kitchen")


@socketio.on('message')
def handle_message(message):
    print "handle_message({})".format(message)
    send(message)

@socketio.on('json')
def handle_json(json):
    print "handle_json({})".format(json)
    send(json, json=True)

@socketio.on('my event')
def handle_my_custom_event(json):
    print "handle_my_custom_event({})".format(json)
    emit('my response', json)

if __name__ == '__main__':
    socketio.run(app)