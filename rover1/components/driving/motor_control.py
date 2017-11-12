import time

class MotorController(object):

  _adapter = None
  _currentFwdBwdSetting = None
  _currentLeftRightSetting = None
  _lastUpdate = None
  _MIN_POWER = -80
  _MAX_POWER = 80

  def __init__(self,motor_controller_adapter):
    if motor_controller_adapter is None:
      raise AttributeError

    self._adapter = motor_controller_adapter
    self._currentFwdBwdSetting = 0
    self._currentLeftRightSetting = 0
    self._lastUpdate = time.time()

  @property
  def currentFwdBwdSetting(self):
    return self._currentFwdBwdSetting

  @property
  def currentLeftRightSetting(self):
    return self._currentLeftRightSetting

  def goForward(self,power_percent=0):
    self._adapter.goForward(power_percent)
    self._currentFwdBwdSetting = max(self._MIN_POWER,min(self._MAX_POWER,power_percent))

  def goBackward(self,power_percent=0):
    self._adapter.goBackward(power_percent)
    self._currentFwdBwdSetting = max(self._MIN_POWER,min(self._MAX_POWER,power_percent))

  def goRight(self,power_percent=0):
    self._adapter.goRight(power_percent)
    self._currentLeftRightSetting = max(self._MIN_POWER,min(self._MAX_POWER,power_percent))

  def goLeft(self,power_percent=0):
    self._adapter.goLeft(power_percent)
    self._currentLeftRightSetting = max(self._MIN_POWER,min(self._MAX_POWER,power_percent))

  def goStraight(self):
    self._adapter.goStraight()

  def stop(self):
    self._adapter.stop()
    self._currentFwdBwdSetting = 0
    self._currentLeftRightSetting = 0

  def adjustFwdBwdSetting(self,power_change=0):
    self._currentFwdBwdSetting = max(self._MIN_POWER,min(self._MAX_POWER,self._currentFwdBwdSetting + power_change ))
    if self._currentFwdBwdSetting >= 0:
      self._adapter.goForward(self._currentFwdBwdSetting)
    else:
      self._adapter.goBackward(abs(self._currentFwdBwdSetting))
  
  def adjustLeftRightSetting(self,power_change=0):
    self._currentLeftRightSetting = max(self._MIN_POWER,min(self._MAX_POWER,self._currentLeftRightSetting + power_change ))
    if self._currentLeftRightSetting >= 0:
      self._adapter.goRight(self._currentLeftRightSetting)
    else:
      self._adapter.goLeft(abs(self._currentLeftRightSetting))

