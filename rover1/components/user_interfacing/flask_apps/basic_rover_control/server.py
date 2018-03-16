# This script derived from Adafruit BNO055 WebGL Example
#
# Requires the flask web framework to be installed.  See http://flask.pocoo.org/
# for installation instructions, however on a Linux machine like the Raspberry
# Pi or BeagleBone black you can likely install it by running:
#  sudo apt-get update
#  sudo apt-get install python-pip
#  sudo pip install flask
#
# Copyright (c) 2017 Mike Chalson
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# Copyright (c) 2015 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
from __future__ import division
import sys
import inspect
import os
from flask import Flask, render_template

# add the main rover source tree to sys.path so modules are accessible - 
rover1_path = os.sep.join(os.path.dirname(os.path.abspath(inspect.stack()[0][1])).split(os.sep)[:-4])
if rover1_path not in sys.path:
    sys.path.insert(0,rover1_path)

import peripherals.sabertooth.sabertooth_adapter as motor
import components.driving.motor_control as control


# - Define some global variables and constants -
MOTOR_FWD_BWD_STEP_SIZE = 10.0
MOTOR_LEFT_RIGHT_STEP_SIZE = 5.0
motor_controller = None

# Create flask application.
app = Flask(__name__)

@app.before_first_request
def do_before_first_request():
    # Do something right before the first request is served.  This is
    # necessary because in debug mode flask will start multiple main threads so
    # this is the only spot to put code that can only run once after starting.
    # See this SO question for more context:
    #   http://stackoverflow.com/questions/24617795/starting-thread-while-running-flask-with-debug

    # Instantiate the motor controller with global scope
    global motor_controller
    motor_controller = control.MotorController(motor.SabertoothPacketizedAdapterGPIO())

@app.route('/stop_driving', methods=['POST'])
def stop_driving():
    global motor_controller
    motor_controller.stop()
    return 'OK'

@app.route('/increase_speed', methods=['POST'])
def increase_speed():
    global motor_controller 
    if motor_controller.currentFwdBwdSetting == 0:
        motor_controller.goStraight()
    motor_controller.adjustFwdBwdSetting(power_change=MOTOR_FWD_BWD_STEP_SIZE)
    return 'OK'

@app.route('/decrease_speed', methods=['POST'])
def decrease_speed():
    global motor_controller 
    if motor_controller.currentFwdBwdSetting == 0:
        motor_controller.goStraight()
    motor_controller.adjustFwdBwdSetting(power_change= -1 * MOTOR_FWD_BWD_STEP_SIZE)
    return 'OK'

@app.route('/turn_more_right', methods=['POST'])
def turn_more_right():
    global motor_controller 
    if motor_controller.currentFwdBwdSetting == 0:
        motor_controller.goStraight()
    motor_controller.adjustLeftRightSetting(power_change=MOTOR_LEFT_RIGHT_STEP_SIZE)
    return 'OK'

@app.route('/turn_more_left', methods=['POST'])
def turn_more_left():
    global motor_controller 
    if motor_controller.currentFwdBwdSetting == 0:
        motor_controller.goStraight()
    motor_controller.adjustLeftRightSetting(power_change= -1 * MOTOR_LEFT_RIGHT_STEP_SIZE)
    return 'OK'

@app.route('/stop_turning', methods=['POST'])
def stop_turning():
    global motor_controller 
    motor_controller.goStraight()
    return 'OK'

@app.route('/')
def root():
    return render_template('index.html')


if __name__ == '__main__':
    # Could enable debug mode for better error messages and live
    # reloading of the server on changes.  Also make the server threaded
    # so multiple connections can be processed at once (very important
    # for using server sent events).
    app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
