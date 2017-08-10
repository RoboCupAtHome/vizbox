FROM ros:kinetic-ros-core

RUN apt-get update && apt-get install -y \
    ros-kinetic-rospy \
    ros-kinetic-std-msgs \
    python-pip \
    && rm -rf /var/lib/apt/lists/*

ADD requirements.txt /

RUN pip install -r requirements.txt

ADD templates /templates
ADD static /static
ADD vizbox.py /

EXPOSE 8888

CMD [ "python", "./vizbox.py" ]