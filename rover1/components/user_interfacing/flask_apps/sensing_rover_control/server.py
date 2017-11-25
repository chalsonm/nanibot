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
import json
import logging
import threading
import time
import sys
import os
import inspect
from flask import Flask, Response, render_template

# add the main rover source tree to sys.path so modules are accessible - 
rover1_path = rover1_path = os.sep.join(os.path.dirname(os.path.abspath(inspect.stack()[0][1])).split(os.sep)[:-4])
if rover1_path not in sys.path:
    sys.path.insert(0,rover1_path)

import peripherals.bno055_imu.inertial_sensor_bno055 as imu
import peripherals.sabertooth.sabertooth_adapter as motor
import components.tracking.state_estimation as state
import components.driving.motor_control as control

# - Define some global variables and constants -
SENSOR_UPDATE_FREQ_HZ = 2.0
SENSOR_CALIBRATION_UPDATE_FREQ_HZ = 1.0
MOTOR_FWD_BWD_STEP_SIZE = 10.0
MOTOR_LEFT_RIGHT_STEP_SIZE = 5.0
heading_estimator = None
motor_controller = None

# Create flask application.
app = Flask(__name__)

def unwrap_heading(heading):
    if heading > 180 and heading <= 360:
        heading = heading - 360
    return heading

def bno_sse():
    """Function to handle sending BNO055 sensor data to the client web browser
    using HTML5 server sent events (aka server push).  This is a generator function
    that flask will run in a thread and call to get new data that is pushed to
    the client web page.
    """
    # Loop forever waiting for a new BNO055 sensor reading and sending it to
    # the client.  Since this is a generator function the yield statement is
    # used to return a new result.
    global heading_estimator
    while True:
        #time.sleep(1.0 / SENSOR_UPDATE_FREQ_HZ)

        latest_estimate = heading_estimator.getCurrentState(wait_for_newest=True)

        # Send the data to the connected client in HTML5 server sent event format.
        data = {'heading': unwrap_heading(latest_estimate['heading']), 'time': latest_estimate['validity_time']}
        yield 'data: {0}\n\n'.format(json.dumps(data))

def bno_calibration_sse():
    """Function to handle sending BNO055 sensor data to the client web browser
    using HTML5 server sent events (aka server push).  This is a generator function
    that flask will run in a thread and call to get new data that is pushed to
    the client web page.
    """
    # Loop forever waiting for a new BNO055 sensor reading and sending it to
    # the client.  Since this is a generator function the yield statement is
    # used to return a new result.
    global heading_estimator
    while True:
        time.sleep(1.0 / SENSOR_CALIBRATION_UPDATE_FREQ_HZ)

        cal_data = heading_estimator.getCurrentCalibration(wait_for_newest=False)

        # Send the data to the connected client in HTML5 server sent event format.
        data = {
            'calSys': cal_data['calibration_sys'], 
            'calGyro': cal_data['calibration_gyro'], 
            'calAccel': cal_data['calibration_accel'], 
            'calMag': cal_data['calibration_mag']}
        yield 'data: {0}\n\n'.format(json.dumps(data))

@app.before_first_request
def do_before_first_request():
    # Do something right before the first request is served.  This is
    # necessary because in debug mode flask will start multiple main threads so
    # this is the only spot to put code that can only run once after starting.
    # See this SO question for more context:
    #   http://stackoverflow.com/questions/24617795/starting-thread-while-running-flask-with-debug

    # Instantiate the IMU heading sensor with global scope
    global heading_estimator
    heading_estimator = state.HeadingEstimator(
        imu.MultiprocessHeadingSensorBNO055(
            sensor_update_frequency_hz=SENSOR_UPDATE_FREQ_HZ))
    
    # Instantiate the motor controller with global scope
    global motor_controller
    motor_controller = control.MotorController(motor.SabertoothPacketizedAdapterGPIO())

@app.route('/bno')
def bno_path():
    # Return SSE response and call bno_sse function to stream sensor data to
    # the webpage.
    return Response(bno_sse(), mimetype='text/event-stream')

@app.route('/bno_calibration')
def bno_calibration_path():
    # Return SSE response and call bno_calibration_sse function to stream sensor calibration
    # data to the webpage.
    return Response(bno_calibration_sse(), mimetype='text/event-stream')

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
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
