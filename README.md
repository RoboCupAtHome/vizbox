RoboCup@Home VizBox
===================

The RoboCup@Home VizBox is a little webserver @Home robots can run to vizualize what is going on. 

The main page shows
- an outline of the current challenge and where the robot is in the story of that challenge. 
- Subtitles of what the robot and operator just said; their conversation
- Images of what the robot sees or a visualisation of the robot's world model, eg. camera images, it's map, anything to make clear what is going on to the audience. 

Additionally, the server accepts HTTP POSTs to which a command sentence can be submitted on /command

![Screenshot](https://github.com/LoyVanBeek/vizbox/blob/master/screenshot.png)

Backends
--------

The server abstracts over the underlying robot via a Backend. A backend accepts messages from the robot's internals. A message is forwarded to the web page via websockets.

Currently, only a ROS backend is implemented:

### Subscriptions:

* ```~operator_text std_msgs/String``` What the robot has heard the operator say
* ```~robot_text std_msgs/String``` What the robot itself is saying
* ```~challenge_step std_msgs/UInt32``` Active item index in the plan or action sequence the robot is executing. 
* ```~image sensor_msgs/Image``` Image of what the robo sees or anything else interesting to the audience

### Publications
* ```~command std_msgs/String``` Command HTTP POSTed to the robot. 

TODO
----

* Allow robot to push action sequence and challenge name to server. Allows for GPSR action sequences etc.
