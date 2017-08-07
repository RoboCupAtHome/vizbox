#! /usr/bin/envv python

from flask import Flask
from flask import render_template

app = Flask(__name__)

@app.route('/')
def challenge():
    return render_template('challenge.html', 
    	challenge="Help me Carry",
    	story=["Follow to car", "Take bag", "Guide to car"],
    	robot_text="OK, I will bring the bag to the kitchen",
    	operator_text="Bring the bag to the kitchen")