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
from flask import Flask, Response, render_template, request

# configure root logger so messages form other libraries are handled
logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(levelname)s (%(name)s): %(message)s")

# - add the main rover source tree to sys.path so modules are accessible - 
rover1_path = os.sep.join(os.path.dirname(os.path.abspath(inspect.stack()[0][1])).split(os.sep)[:-4])
if rover1_path not in sys.path:
    sys.path.insert(0,rover1_path)

# - import project modules
import peripherals.bno055_imu.inertial_sensor_bno055 as imu
import peripherals.sabertooth.sabertooth_adapter as motor
import components.tracking.state_estimation as state
import components.driving.motor_control as control
import components.driving.feedback_control as feedback


# - Define some global variables and constants -
SENSOR_UPDATE_FREQ_HZ = 2.0
SENSOR_CALIBRATION_UPDATE_FREQ_HZ = 1.0
FEEDBACK_CONTROL_UPDATE_INTERVAL_SEC = 0.1
FEEDBACK_UI_REPORTING_INTERVAL_SEC = 2.0
FEEDBACK_CONTROLLER_P_GAIN = 4.0
MOTOR_FWD_BWD_STEP_SIZE = 10.0
MOTOR_LEFT_RIGHT_STEP_SIZE = 5.0
imu_sensor = None
heading_estimator = None
motor_controller = None
feedback_control_system = None

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
        # Note: sample rate of sensor will determine how long this function call blocks
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

def feedback_control_system_sse():
    """Function to handle sending feedback control system data to the client web browser
    using HTML5 server sent events (aka server push).  This is a generator function
    that flask will run in a thread and call to get new data that is pushed to
    the client web page.
    """
    # Loop forever waiting for a new BNO055 sensor reading and sending it to
    # the client.  Since this is a generator function the yield statement is
    # used to return a new result.
    global feedback_control_system
    while True:
        time.sleep(FEEDBACK_UI_REPORTING_INTERVAL_SEC)

        try:
            # Send the data to the connected client in HTML5 server sent event format.
            observation = feedback_control_system.get_corrected_observed_state()
            data = {
                'setPoint': feedback_control_system.set_point, 
                'observedHeading': observation['heading'],
                'observationTime': observation['validity_time']}
            #data = {'setPoint': 123}
            yield 'data: {0}\n\n'.format(json.dumps(data))
        except:
            # This exception probably occurred because the rover is in Manual mode
            pass

def run_feedback_control_loop():
    logging.getLogger(__name__).info("initiating feedback control loop") 
    global feedback_control_system

    while True:
        time.sleep(FEEDBACK_CONTROL_UPDATE_INTERVAL_SEC)
        try:
            if feedback_control_system is None:
                continue
            feedback_control_system.update_plant_command()
        except:
            pass

@app.before_first_request
def do_before_first_request():
    # Do something right before the first request is served.  This is
    # necessary because in debug mode flask will start multiple main threads so
    # this is the only spot to put code that can only run once after starting.
    # See this SO question for more context:
    #   http://stackoverflow.com/questions/24617795/starting-thread-while-running-flask-with-debug

    # Instantiate the IMU heading sensor and heading estimator with global scope
    global imu_sensor
    global heading_estimator

    imu_sensor = imu.MultiprocessHeadingSensorBNO055(
        sensor_update_frequency_hz=SENSOR_UPDATE_FREQ_HZ)
    heading_estimator = state.HeadingEstimator(imu_sensor)
    
    # Instantiate the motor controller with global scope
    global motor_controller
    motor_controller = control.MotorController(motor.SabertoothPacketizedAdapterGPIO())
     
    feedback_control_thread = threading.Thread(
      target=run_feedback_control_loop,
      args=())
    feedback_control_thread.daemon = True  # Don't let the thread block exiting.
    feedback_control_thread.start()

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

@app.route('/heading_seeker')
def heading_seeker_path():
    # Return SSE response and call feedback_control_system_sse function to stream heading seeker
    # data to the webpage.
    return Response(feedback_control_system_sse(), mimetype='text/event-stream')

@app.route('/load_calibration', methods=['POST'])
def load_calibration():
    global imu_sensor
    imu_sensor.load_calibration()
    return 'OK'

@app.route('/save_calibration', methods=['POST'])
def save_calibration():
    global imu_sensor
    imu_sensor.save_calibration()
    return 'OK'

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

@app.route('/start_feedback_control_system', methods=['POST'])
def start_feedback_control_system():
    global motor_controller 
    global heading_estimator 
    global feedback_control_system

    if feedback_control_system is not None:
        return 'OK'

    # Lookup current heading
    state = heading_estimator.getCurrentState()
    initial_state = state['heading']
    #initial_state = 0
    # TODO: add getter to motor_controller
    initial_forward_speed = motor_controller._currentFwdBwdSetting

    feedback_control_system = feedback.HeadingFeedbackController(
        heading_estimator, # observer
        motor_controller, # motor_controller
        None, # update_interval_sec
        FEEDBACK_CONTROLLER_P_GAIN, #proportional_gain
        0.0, # integral_gain
        measurement_offset=0,
        initial_state=initial_state,
        nominal_forward_power=initial_forward_speed,
        verbose=False)
    return 'OK'

@app.route('/suspend_feedback_control_system', methods=['POST'])
def suspend_feedback_control_system():
    global feedback_control_system
    feedback_control_system = None
    return 'OK'

@app.route('/adjust_feedback_control_setpoint', methods=['POST'])
def adjust_feedback_control_setpoint():
    global feedback_control_system
    adjustment = float(request.form['adjustment']) 
    logging.getLogger(__name__).info("adjust setpoint by: {}".format(adjustment)) 
    feedback_control_system.update_set_point(adjustment + feedback_control_system.set_point)
    return 'OK'

@app.route('/adjust_feedback_control_setpoint_to_zero', methods=['POST'])
def adjust_feedback_control_setpoint_to_zero():
    global feedback_control_system
    logging.getLogger(__name__).info("adjust setpoint to 0") 
    feedback_control_system.update_set_point(0)
    return 'OK'

@app.route('/reset_feedback_control_measurement_offset', methods=['POST'])
def reset_feedback_control_measurement_offset():
    global feedback_control_system
    heading = feedback_control_system.get_observed_state()['heading']
    logging.getLogger(__name__).info("set measurement offset to {}".format(heading)) 
    feedback_control_system.update_measurement_offset(heading)
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

