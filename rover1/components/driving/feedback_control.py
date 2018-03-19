"""
This module is a collection of classes and functions related to simple, classical feedback control systems
"""

from __future__ import division
import time
import logging
import multiprocessing as multiproc
from copy import deepcopy

import peripherals.bno055_imu.inertial_sensor_bno055 as imu
import peripherals.sabertooth.sabertooth_adapter as sabertooth_adapter
import components.driving.motor_control as motor_control
import components.tracking.state_estimation as state_estimation


logging.getLogger(__name__).addHandler(logging.NullHandler())


class FeedbackControlSystemManager(object):
    
    def __init__(self):
        self._control_loop_stop_flag = multiproc.Value('b',0)
        self._set_point = multiproc.Value('f',0.0)
        
    @property
    def control_loop_stop_flag(self):
        return self._control_loop_stop_flag
    
    @property
    def set_point(self):
        return self._set_point
    
    def update_set_point(self,new_set_point):
        self._set_point.value = new_set_point
    
    def stop_control_loop(self):
        self._control_loop_stop_flag.value = 1
        self._feedback_iteration_proc.join(10)
    
    def launch_heading_feedback_control_system(self):
        
        # Make sure stop flag is initialized
        self._clear_control_loop_stop_flag()
        
        self._feedback_iteration_proc = multiproc.Process(
            target=self._heading_feedback_control_loop,
            args=(
                self.control_loop_stop_flag, # stop_control_loop
                self.set_point, # heading_set_point
                0.1)) # update_interval_sec
        #self._feedback_iteration_proc.daemon = True  # Don't let the BNO reading thread block exiting.
        self._feedback_iteration_proc.start()
        
    def _clear_control_loop_stop_flag(self):
        self._control_loop_stop_flag.value = 0
    
    def _heading_feedback_control_loop(
        self,
        stop_control_loop,
        heading_set_point,
        update_interval_sec):
        
        sensor_update_frequency_hz = 10.0
        
        # - Construct Feedback Control System Components
        motorControllerAdapter = sabertooth_adapter.SabertoothPacketizedAdapterGPIO()
        motor_controller = motor_control.MotorController(motorControllerAdapter)
        heading_sensor = imu.MultiprocessHeadingSensorBNO055(
            sensor_update_frequency_hz=sensor_update_frequency_hz)
        heading_estimator = state_estimation.HeadingEstimator(heading_sensor)

        feedback_controller = HeadingFeedbackController(
            heading_estimator, # observer
            motor_controller, # motor_controller
            update_interval_sec, # update_interval_sec
            0.4, #proportional_gain
            0.0, # integral_gain
            measurement_offset=0,
            initial_state=heading_set_point.value,
            nominal_forward_power=25,
            verbose=False)
        
        while stop_control_loop.value == 0:
            # TODO: add mutex around this
            feedback_controller.update_set_point(heading_set_point.value)
            feedback_controller.update_plant_command()
            time.sleep(update_interval_sec)
        
        motor_controller.stop()
        raise ValueError('Stopped updating feedback controller because of stop condition.')
        
        

class HeadingFeedbackController(object):
    
    _observer_timeout_sec = 5
    
    def __init__(
        self,
        observer,
        motor_controller,
        update_interval_sec,
        proportional_gain,
        integral_gain,
        measurement_offset=0,
        initial_state=0,
        nominal_forward_power=0,
        verbose=False):
        
        self._verbose = verbose
        self._break_feedback_iterator = False
        
        # Set the interval on which the feedback controller will try to make updates
        self._update_interval_sec = update_interval_sec
        # Set heading observer
        self._observer = observer
        # Set motor controller and ensure that we're initially not driving
        self._motor_controller = motor_controller
        self._nominal_forward_power = float(nominal_forward_power)
        self._motor_controller.stop()
        self._driving = False
        
        # Set the feedback gains
        self._P_gain = float(proportional_gain)
        self._I_gain = float(integral_gain)
        
        # Set the measurement calibration offset
        self._measurement_offset = float(measurement_offset)
        
        # Initialize last observer time as current time
        self._last_observer_update_time = time.time()
        # Initialize last controller update time as current time
        self._last_controller_update_time = time.time()
        # Keep track of initial invocation time, for logging and debugging
        self._start_time = time.time()
        
        self._set_point = float(initial_state)
        self._last_heading_estimate = self._set_point
        self._last_plant_command = self._set_point
        self._cumulative_error = 0.0
        
        
    @property
    def set_point(self):
        return self._set_point
    
    @property
    def measurement_offset(self):
        return self._measurement_offset
    
    def update_set_point(self, set_point):
        """TODO: add safety limits"""
        self._set_point = float(set_point) % 360.0
        
    def update_measurement_offset(self, offset):
        """TODO: add safety limits"""
        self._measurement_offset = float(offset) % 360.0
    
    def get_observed_state(self):
        return self._observer.getCurrentState()

    def get_corrected_observed_state(self):
        """
        Subtract measurement offsets from observed state then return the result
        Note: at this time, measurement offset is a scalar, representing the heading offset only
        """
        result = deepcopy(self._observer.getCurrentState())
        result['heading'] -= self._measurement_offset
        return result
    
    def update_plant_command(self):
        """
        Attempt to update the motor controller command, send stop command if something goes wrong
        
        """
        try:
            # If we're not yet driving, send nominal forward command to motor controller
            if not self._driving:
                # Set nominal forward power on motor controller
                self._motor_controller.goForward(power_percent=self._nominal_forward_power)
                self._driving = True
            
            new_plant_command = self._get_new_plant_command()
            old_plant_command = self._motor_controller.currentLeftRightSetting
            heading_change = new_plant_command - old_plant_command
            self._motor_controller.adjustLeftRightSetting(heading_change)
        except:
            logging.getLogger(__name__).warn("Failed to update plant command.  Sending stop command to motor controller.")
            self._motor_controller.stop()
            raise
        
    def _get_control_error(self,measurement,set_point):
        measurement -= 180
        set_point -= 180
        diff = set_point - measurement
        if diff > 180:
            return diff - 360
        elif diff < -180:
            return 360 + diff
        else:
            return diff
        
    def _get_new_plant_command(self):
        # Compute new motor command from proportional feedback
        corrected_observed_state = self.get_corrected_observed_state()
        state_validity_time = corrected_observed_state['validity_time']
        
        # If observer data is too old, raise exception that should shut down the motors
        current_time = time.time()
        if current_time - state_validity_time >= self._observer_timeout_sec:
            error_str_details = "{0:.1f} (current) - {1:.1f} (validity) = {2:.1f} (diff)".format(
                current_time,
                state_validity_time,
                current_time-state_validity_time)
            raise ValueError('observer data has become too stale.\n\t'+error_str_details)
            
        # Make sure there's a valid set point and heading measurement 
        # - Should raise ValueError otherwise
        gain = float(self._P_gain)
        set_point = float(self.set_point)
        corrected_heading_estimate = corrected_observed_state['heading']
        
        # --- Proportional Error Feedback ---
        new_plant_command = gain * self._get_control_error(corrected_heading_estimate, set_point)
        if self._verbose:
            print '\n\ntime: {}\nset point: {}\ncalibrated observer heading: {}, \nnew command: {}'.format(
                current_time - self._start_time,
                set_point,
                corrected_heading_estimate,
                new_plant_command)
            
        return new_plant_command

