"""
MIT License

Copyright (c) 2017 Nanigans

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

BNO055.py Absolute Orientation Sensor Python Library is copyright (c) 2015 Adafruit 
Industries and released under a MIT License.  See more details at:
  https://github.com/adafruit/Adafruit_Python_BNO055/blob/master/LICENSE

"""

from Adafruit_BNO055 import BNO055
import time
import threading
import multiprocessing as multiproc
import logging
import os
import json
import inspect


CALIBRATION_FILE_PATH = 'calibration'
CALIBRATION_FILE_NAME = 'bno_calibration.json'
EXECUTION_PATH = os.path.dirname(os.path.abspath(inspect.stack()[0][1]))

logging.getLogger(__name__).addHandler(logging.NullHandler())

def read_bno_threaded(bno, update_frequency_hz, bno_changed, bno_data):
    """Function to read the BNO sensor and update the bno_data object with the
    latest BNO orientation, etc. state.  Must be run in its own thread because
    it will never return!
    Pretty much copied from Adafruit https://github.com/adafruit/Adafruit_Python_BNO055.git
    """
    bno_data['update_count'] = 0
    while True:
    
        # Grab new BNO sensor readings.
        update_time = time.time()
        #temp = bno.read_temp()
        linX, linY, linZ = bno.read_linear_acceleration()
        #heading, roll, pitch = bno.read_euler()
        #x, y, z, w = bno.read_quaternion()
        #sys, gyro, accel, mag = bno.get_calibration_status()
        status, self_test, error = bno.get_system_status(run_self_test=False)
        if error != 0:
            print 'Error! Value: {0}'.format(error)
        # Capture the lock on the bno_changed condition so the bno_data shared
        # state can be updated.
        with bno_changed:
            bno_data['update_count'] += 1
            bno_data['update_time'] = update_time
            bno_data['linear_accelerations'] = (linX, linY, linZ)
            #bno_data['euler'] = (heading, roll, pitch)
            #bno_data['temp'] = temp
            #bno_data['quaternion'] = (x, y, z, w)
            #bno_data['calibration'] = (sys, gyro, accel, mag)
            # Notify any waiting threads that the BNO state has been updated.
            bno_changed.notify_all()
        # Sleep until the next reading.
        time.sleep(1.0/update_frequency_hz)


def read_bno_multiproc(bno, update_frequency_hz, shared_data, condition):
    """Function to read the BNO sensor and update the shared memory array shared_data
    shared_data[0] := update count
    shared_data[1] := update time
    shared_data[2] := linear acceleration X
    shared_data[3] := linear acceleration Y
    shared_data[4] := linear acceleration Z
    """
    # update count
    shared_data[0] = 0
    while True:
    
        # Grab new BNO sensor readings.
        update_time - time.time()
        #temp = bno.read_temp()
        linX, linY, linZ = bno.read_linear_acceleration()
        #heading, roll, pitch = bno.read_euler()
        #x, y, z, w = bno.read_quaternion()
        #sys, gyro, accel, mag = bno.get_calibration_status()
        status, self_test, error = bno.get_system_status(run_self_test=False)
        if error != 0:
            print 'Error! Value: {0}'.format(error)
        
        # Safely update all shared state with synchronization
        with condition:
            shared_data[0] += 1
            shared_data[1] = update_time
            shared_data[2] = linX
            shared_data[3] = linY
            shared_data[4] = linZ
            # Notify any listening threads that new data is available
            condition.notify_all()
        
        # Sleep until the next reading.
        time.sleep(1.0/update_frequency_hz)


def read_bno_heading_multiproc(bno, update_frequency_hz, shared_data, condition):
    """Function to read the BNO sensor and update the shared memory array shared_data
    shared_data[0] := update count
    shared_data[1] := update time
    shared_data[2] := heading
    shared_data[3] := sys
    shared_data[4] := gyro
    shared_data[5] := accel
    shared_data[6] := mag 
    """
    # update count
    shared_data[0] = 0
    while True:
    
        # Grab new BNO sensor readings.
        update_time = time.time()
        #temp = bno.read_temp()
        #linX, linY, linZ = bno.read_linear_acceleration()
        heading, roll, pitch = bno.read_euler()
        #x, y, z, w = bno.read_quaternion()
        sys, gyro, accel, mag = bno.get_calibration_status()
        #status, self_test, error = bno.get_system_status(run_self_test=False)
        #if error != 0:
        #    print 'Error! Value: {0}'.format(error)
    
        # Safely update all shared state with synchronization
        with condition:
            shared_data[0] += 1
            shared_data[1] = update_time 
            shared_data[2] = heading
            shared_data[3] = sys
            shared_data[4] = gyro
            shared_data[5] = accel
            shared_data[6] = mag
            # Notify any listening threads that new data is available
            condition.notify_all()
        
        # Sleep until the next reading.
        time.sleep(1.0/update_frequency_hz)


class InertialSensorBNO055:

  _sensor = None

  def __init__(self,calibration_data=None,axis_remap=None):

    # Create and configure the BNO sensor connection.
    # Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
    self._sensor = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)

    if not self._sensor.begin():
      raise RuntimeError('Failed to initialize BNO055!')
    if axis_remap is not None:
      self._sensor.set_axis_remap(**axis_remap)
    if calibration_data is not None:
      self._sensor.set_calibration(calibration_data)

  def get_measurement(self):
    # Grab new BNO sensor readings.
    linX, linY, linZ = self._sensor.read_linear_acceleration()
    status, self_test, error = self._sensor.get_system_status(run_self_test=False)
    if error != 0:
      raise RuntimeError('Sensor error in BNO055')
    
    result = {}
    result['validity_time'] = time.time()
    result['acceleration_x'] = linX
    result['acceleration_y'] = linY
    result['acceleration_z'] = linZ
    
    return result

class ThreadedInertialSensorBNO055:

  BNO_UPDATE_FREQUENCY_HZ = 100.0
  _sensor = None
  _condition = None
  _state = {}

  def __init__(self,calibration_data=None,axis_remap=None):

    # Create and configure the BNO sensor connection.
    # Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
    self._sensor = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)

    if not self._sensor.begin():
      raise RuntimeError('Failed to initialize BNO055!')
    if axis_remap is not None:
      self._sensor.set_axis_remap(**axis_remap)
    if calibration_data is not None:
      self._sensor.set_calibration(calibration_data)

    self._condition = threading.Condition()

    sensor_thread = threading.Thread(
      target=read_bno_threaded,
      args=(
        self._sensor,
        self.BNO_UPDATE_FREQUENCY_HZ,
        self._condition,
        self._state))
    sensor_thread.daemon = True  # Don't let the BNO reading thread block exiting.
    sensor_thread.start()

  def get_measurement(self):
     
    with self._condition:
      result = {}
      result['lookup_time'] = time.time()
      result['update_time'] = self._state['update_time']
      result['update_count'] = self._state['update_count']
      result['acceleration_x'] = self._state['linear_accelerations'][0]
      result['acceleration_y'] = self._state['linear_accelerations'][1]
      result['acceleration_z'] = self._state['linear_accelerations'][2]
      return result


class MultiprocessInertialSensorBNO055(object):

  def __init__(self,calibration_data=None,axis_remap=None,sensor_update_frequency_hz=20.0):

    # Initialize BNO sensor with up to one retry
    try:
        # Create and configure the BNO sensor connection.
        # Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
        self._sensor = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
        self._sensor.begin()
    except RuntimeError:
        # Second try, just in case
        self._sensor = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
        self._sensor.begin()

    if axis_remap is not None:
      self._sensor.set_axis_remap(**axis_remap)
    if calibration_data is not None:
      self._sensor.set_calibration(calibration_data)

    # The 5 elements of the shared memory array are (update count, update time, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z)
    self._multiproc_shared_data = multiproc.Array('d',5)
    self._condition = multiproc.Condition()
    self._state = {}
    self.sensor_update_frequency_hz = sensor_update_frequency_hz

    # TODO: it's probably bad form to spawn a process within the constructor of an object.  Could move this
    #       to a public start() method
    sensor_proc = multiproc.Process(
      target=read_bno_multiproc,
      args=(
        self._sensor,
        self.sensor_update_frequency_hz,
        self._multiproc_shared_data,
        self._condition))
    sensor_proc.daemon = True  # Don't let the BNO reading thread block exiting.
    sensor_proc.start()

  def get_last_measurement(self):
    """
    Safely lookup and return most recent version of all measurement attributes, with synchronization 
    """
      
    # Grab lock to make sure all attributes are synchronized
    with self._condition:
      result = {}
      result['lookup_time'] = time.time()
      result['update_count'] = self._multiproc_shared_data[0]
      result['update_time'] = self._multiproc_shared_data[1]
      result['acceleration_x'] = self._multiproc_shared_data[2]
      result['acceleration_y'] = self._multiproc_shared_data[3]
      result['acceleration_z'] = self._multiproc_shared_data[4]
      return result

  def get_next_measurement(self):
    """
    Wait for and return next version of all measurement attributes, with synchronization 
    """
     
    # Grab lock to make sure all attributes are synchronized
    with self._condition:
      self._condition.wait()
      result = {}
      result['lookup_time'] = time.time()
      result['update_count'] = self._multiproc_shared_data[0]
      result['update_time'] = self._multiproc_shared_data[1]
      result['acceleration_x'] = self._multiproc_shared_data[2]
      result['acceleration_y'] = self._multiproc_shared_data[3]
      result['acceleration_z'] = self._multiproc_shared_data[4]
      return result


class MultiprocessHeadingSensorBNO055(object):

  def __init__(self, calibration_data=None, axis_remap=None, sensor_update_frequency_hz=20.0):

    # Initialize BNO sensor with up to one retry
    try:
        # Create and configure the BNO sensor connection.
        # Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
        self._sensor = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
        self._sensor.begin()
    except RuntimeError:
        # Second try, just in case
        self._sensor = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
        self._sensor.begin()

    if not self._sensor.begin():
      raise RuntimeError('Failed to initialize BNO055!')
    if axis_remap is not None:
      self._sensor.set_axis_remap(**axis_remap)
    if calibration_data is not None:
      self._sensor.set_calibration(calibration_data)

    # The 7 elements of the shared memory array are (update count, update time, heading, sys, gyro, accel, mag)
    self._multiproc_shared_data = multiproc.Array('d',7)
    self._condition = multiproc.Condition()
    self._state = {}
    self.sensor_update_frequency_hz = sensor_update_frequency_hz

    # TODO: it's probably bad form to spawn a process within the constructor of an object.  Could move this
    #       to a public start() method
    sensor_proc = multiproc.Process(
      target=read_bno_heading_multiproc,
      args=(
        self._sensor,
        self.sensor_update_frequency_hz,
        self._multiproc_shared_data,
        self._condition))
    sensor_proc.daemon = True  # Don't let the BNO reading thread block exiting.
    sensor_proc.start()
    self._process = sensor_proc

  def shutdown(self,timeout=10):
    self._process.join(timeout)

  def load_calibration(self):
    """Load calibration data from disk and apply it to the sensor"""
    
    load_file = os.sep.join([EXECUTION_PATH, CALIBRATION_FILE_PATH, CALIBRATION_FILE_NAME])

    # - Load the calibration data from disk
    try:
      with open(load_file, 'r') as file:
        data = json.load(file)
      # Grab the lock and set calibration data for the sensor
      with self._condition:
        data = self._sensor.set_calibration(data)
    except IOError as e:
      if e.errno == 2:
        # Reasonable error to occur if there has never been a cal file saved
        logging.getLogger(__name__).warn("Failed to load imu sensor calibration data.  Proceeding anyway.")
        return
      else:
        raise
    except:
      # Unexpected error.  Abandon ship!
      raise

  def save_calibration(self):
    """Get calibration data from sensor and save to disk for future use"""
    
    save_file = os.sep.join([EXECUTION_PATH, CALIBRATION_FILE_PATH, CALIBRATION_FILE_NAME])

    try:
      # Grab the lock and get calibration data from the sensor
      with self._condition:
        data = self._sensor.get_calibration()
      # Save the calibration data to disk
      with open(save_file, 'w') as file:
        json.dump(data, file)
    except IOError as e:
      # If saving calibration data fails for this reason, warn and try to proceed
      logging.getLogger(__name__).warn("Failed to save imu sensor calibration data.  Proceeding anyway.")
      return
    except:
      # Unexpected error.  Abandon ship!
      raise

  def get_last_measurement(self):
    """
    Safely lookup and return most recent version of all measurement attributes, with synchronization 
    """
      
    # Grab lock to make sure all attributes are synchronized
    with self._condition:
      result = {}
      result['lookup_time'] = time.time()
      result['update_count'] = self._multiproc_shared_data[0]
      result['update_time'] = self._multiproc_shared_data[1]
      result['heading'] = self._multiproc_shared_data[2]
      return result

  def get_next_measurement(self):
    """
    Wait for and return next version of all measurement attributes, with synchronization 
    """
     
    # Grab lock to make sure all attributes are synchronized
    with self._condition:
      self._condition.wait()
      result = {}
      result['lookup_time'] = time.time()
      result['update_count'] = self._multiproc_shared_data[0]
      result['update_time'] = self._multiproc_shared_data[1]
      result['heading'] = self._multiproc_shared_data[2]
      return result
  
  def get_last_calibration(self):
    """
    Safely lookup and return most recent version of all calibration attributes, with synchronization 
    """
     
    # Grab lock to make sure all attributes are synchronized
    with self._condition:
      result = {}
      result['lookup_time'] = time.time()
      result['update_time'] = self._multiproc_shared_data[1]
      result['calibration_sys'] = self._multiproc_shared_data[3]
      result['calibration_gyro'] = self._multiproc_shared_data[4]
      result['calibration_accel'] = self._multiproc_shared_data[5]
      result['calibration_mag'] = self._multiproc_shared_data[6]
      return result

  def get_next_calibration(self):
    """
    Wait for and return next version of all calibration attributes, with synchronization 
    """
     
    # Grab lock to make sure all attributes are synchronized
    with self._condition:
      self._condition.wait()
      result = {}
      result['lookup_time'] = time.time()
      result['update_time'] = self._multiproc_shared_data[1]
      result['calibration_sys'] = self._multiproc_shared_data[3]
      result['calibration_gyro'] = self._multiproc_shared_data[4]
      result['calibration_accel'] = self._multiproc_shared_data[5]
      result['calibration_mag'] = self._multiproc_shared_data[6]
      return result
