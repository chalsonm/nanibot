import time
import sys
import logging

class StateEstimator(object):

  def __init__(self,sensor):

    # Enable verbose debug logging if -v is passed as a parameter.
    if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
      logging.basicConfig(level=logging.DEBUG)

    self._sensor = sensor

    self._last_update_time = time.time()
    self._last_acceleration = (0,0,0)
    self._last_velocity = (0,0,0)
    self._last_position = (0,0,0)

  def _updateStateEstimates(self):

    meas = self._sensor.get_measurement()
    t = meas['validity_time']
    x = meas['acceleration_x']
    y = meas['acceleration_y']
    z = meas['acceleration_z']

    # - Compute new state
    dT = t - self._last_update_time
    velocities = (self._last_velocity[0] + dT * x, self._last_velocity[1] + dT * y, self._last_velocity[2] + dT * z)
    positions = (self._last_position[0] + 0.5 * dT * (velocities[0] + self._last_velocity[0]),
                 self._last_position[1] + 0.5 * dT * (velocities[1] + self._last_velocity[1]),
                 self._last_position[2] + 0.5 * dT * (velocities[2] + self._last_velocity[2]))

    # - Update state estimates
    self._last_update_time = t
    self._last_acceleration = (x,y,z)
    self._last_velocity = velocities
    self._last_position = positions

  def getCurrentState(self):
    self._updateStateEstimates()

    return {
      'position':self._last_position,
      'velocity':self._last_velocity,
      'acceleration':self._last_acceleration,
      't':self._last_update_time}

class HeadingEstimator(object):
    
    def __init__(self,sensor):
        
        self._sensor = sensor
        self._last_validity_time = time.time()
        self._last_heading = 0
        self._last_calibration = None
        
    @property
    def last_heading(self):
        return self._last_heading
    
    @property
    def last_validity_time(self):
        return self._last_validity_time
    
    def _updateStateEstimates(self):
        
        meas = self._sensor.get_measurement()
        # - Update state estimates
        self._last_validity_time = meas['update_time']
        self._last_heading = meas['heading']
    
    def _updateCalibrationData(self):
        
        cal = self._sensor.get_calibration()
        # - Update state estimates
        self._last_calibration = cal
        
    def getCurrentState(self):
        self._updateStateEstimates()

        return {
            'heading':self.last_heading,
            'validity_time':self.last_validity_time}
    
    def getCurrentCalibration(self):
        self._updateCalibrationData()

        return self._last_calibration 
