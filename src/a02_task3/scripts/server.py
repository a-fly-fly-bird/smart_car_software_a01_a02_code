#! /usr/bin/env python3

import os
import rospy
import threading
import json

from flask import Flask, render_template, request

from a02_task3.msg import car_info

hello_str = "test"

def ros_callback(msg):
	global hello_str
	hello_str = msg
	print(hello_str.front_light_state)

threading.Thread(target=lambda: rospy.init_node('example_node', disable_signals=True)).start()
rospy.Subscriber('car_info_msg', car_info, ros_callback)

app = Flask(__name__)

@app.route('/index.html')
def hello_world():
	global hello_str
	return render_template('index.html', msg = hello_str)

if __name__ == '__main__':
    app.run(host='0.0.0.0')
