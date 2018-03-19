from __future__ import division
import serial
import pigpio
import time
import logging

######################################################################################################
#
# Adapter for driving the Sabertooth motor controller in packetized mode
#
# The packet format for the Sabertooth consists of an address byte, a command byte, a data byte
# and a seven bit checksum. Address bytes have value greater than 128, and all subsequent bytes
# have values 127 or lower. This allows multiple types of devices to share the same serial line.
#
######################################################################################################

logging.getLogger(__name__).addHandler(logging.NullHandler())

SIM_MODE = False
SHOW_RESULTS_MODE = False
DO_LOOPBACK = False
 
DEFAULT_PORT = '/dev/ttyAMA0'
DEFAULT_BAUD = 9600
DEFAULT_ADDRESS = 131
DEFAULT_GPIO_TX = 25
DEFAULT_GPIO_WORD_OFFSET = 0 # microseconds
DEFAULT_STOP_BITS = 1
CHECKSUM_SALT = 0b01111111
AUTO_BAUD_RATE_COMMAND = 0b10101010

# --- Sabertooth Command Definitions ---
M1_FWD = 0
M1_BWD = 1
MIN_VOLTAGE_COMMAND = 2
M2_FWD = 4
M2_BWD = 5
M1_DRIVE = 6
M2_DRIVE = 7
MIXED_FWD = 8
MIXED_BWD = 9
MIXED_R_TURN = 10
MIXED_L_TURN = 11
MIXED_BWD_FWD = 12
MIXED_LR_TURN = 13

# --- Sabertooth Command Depths
MIXED_FWD_STOP, MIXED_BWD_STOP, MIXED_R_STOP, MIXED_L_STOP, MIXED_LR_STRAIGHT = 64, 64, 64, 64, 64
MIXED_FWD_MIN = 65
MIXED_FWD_MAX = 127
MIXED_BWD_MIN = 63
MIXED_BWD_MAX = 0
MIXED_R_MIN = 65
MIXED_R_MAX = 127
MIXED_L_MIN = 63
MIXED_L_MAX = 0

class SabertoothPacketizedAdapter(object):

  _conn = None
  _serial_address = None

  # - Create and setup the motor controllor adapter
  def __init__(self,port=DEFAULT_PORT,baudrate=DEFAULT_BAUD,serial_address=DEFAULT_ADDRESS):
    if SIM_MODE:
      print 'In sim mode.  Commands will not be sent to TXD.'

    # Create serial connection
    self._conn = serial.Serial(port,baudrate=baudrate)
    # Initialize sabertooth
    #self._conn.flushInput()
    #self._conn.flushOutput()
    time.sleep(3)
    self._write_bytes(bytearray([AUTO_BAUD_RATE_COMMAND]))

    self._serial_address = serial_address

    # Sabertooth requires you to define a min battery voltage upon startup
    self._send_packet(
      self._get_packet_for_command(
        MIN_VOLTAGE_COMMAND,10))

  def _shutdown(self):
    self._conn.close()

  def _write_bytes(self,data_bytes):
    bytes_written = self._conn.write(data_bytes)
    if SIM_MODE:
      print "Commanded Packet ({0})".format(
        str(data_bytes))
        #data_bytes.decode('utf_8'))

    else:
      if SHOW_RESULTS_MODE:
        print "Bytes written: {0}".format(bytes_written)
      if DO_LOOPBACK:
        self._conn.flushOutput()
        return_bytes = self._conn.read(bytes_written)
        print "Loopback data - type: ", type(return_bytes), ", length: ",len(return_bytes), ", raw value: ", ','.join([x for x in return_bytes]), ", unicode representation: ", unicode(return_bytes)


  #---------- Main Sabertooth API Implementation for Packetized Serial ------------

  # -- Build Packet --
  # Packets will always contain 4 bytes, transmitted in the following order:
  #  1 - address: ID of the device on the serial bus
  #  2 - command: ID of the command, defined in the sabertooth API
  #  3 - data: Specific data, who's interpretation depends on the command
  #  4 - checksum: a validation byte, who's interpretation and generation is defined in the sabertooth API
  def _get_packet_for_command(self, command, data):
    checksum = (self._serial_address + int(command) + int(data)) & CHECKSUM_SALT 

    logging.getLogger(__name__).debug("motor command: {address}, {command}, {data}, {check}".format(
      address=self._serial_address,
      command=int(command),
      data=int(data),
      check=checksum)) 

    #packet = {
    #  0:self._serial_address,
    #  1:command,
    #  2:data,
    #  3:checksum}

    packet = bytearray(4)
    packet[0] = self._serial_address
    packet[1] = chr(int(command))
    packet[2] = chr(int(data))
    packet[3] = chr(checksum)
  
    return packet

  # -- Send Packet --
  def _send_packet(self,packet):
    result = self._write_bytes(packet)


  #----- End of Main Sabertooth API Implementation for Packetized Serial ------


  # ---- Main External Methods for Affecting Moter Behavior ----
  def stop(self):
    logging.getLogger(__name__).debug("motor stop")
    self._send_packet(
      self._get_packet_for_command(
        M1_FWD,0))
    self._send_packet(
      self._get_packet_for_command(
        M2_FWD,0))

  def goForward(self,power_percent=0):
    pwr = min(80,max(0,power_percent))
    logging.getLogger(__name__).debug("motor goForward - {}% power".format(pwr))
    command_data = MIXED_FWD_MIN + 0.01 * pwr * (MIXED_FWD_MAX - MIXED_FWD_MIN)
    self._send_packet(self._get_packet_for_command(MIXED_BWD_FWD,command_data))

  def goBackward(self,power_percent=0):
    pwr = min(80,max(0,power_percent))
    logging.getLogger(__name__).debug("motor goBackward - {}% power".format(pwr))
    #command_data = MIXED_BWD_MIN - 0.01 * pwr * (MIXED_FWD_MIN - MIXED_FWD_MAX)
    command_data = MIXED_BWD_MIN - 0.01 * pwr * (MIXED_BWD_MIN - MIXED_BWD_MAX)
    self._send_packet(self._get_packet_for_command(MIXED_BWD_FWD,command_data))

  def goRight(self,power_percent=0):
    pwr = min(80,max(0,power_percent))
    logging.getLogger(__name__).debug("motor goRight - {}% power".format(pwr))
    command_data = MIXED_R_MIN + 0.01 * pwr * (MIXED_R_MAX - MIXED_R_MIN)
    self._send_packet(self._get_packet_for_command(MIXED_LR_TURN,command_data))
  
  def goLeft(self,power_percent=0):
    pwr = min(80,max(0,power_percent))
    logging.getLogger(__name__).debug("motor goLeft - {}% power".format(pwr))
    command_data = MIXED_L_MIN - 0.01 * pwr * (MIXED_L_MIN - MIXED_L_MAX)
    self._send_packet(self._get_packet_for_command(MIXED_LR_TURN,command_data))

  def goStraight(self):
    logging.getLogger(__name__).debug("motor goStraight")
    self._send_packet(self._get_packet_for_command(MIXED_LR_TURN,MIXED_LR_STRAIGHT))
  # ---- End of Main External Methods for Affecting Moter Behavior ----

  
  #--------------- test scripts --------------

  def execute_timed_commands(self,commands,default_duration):
    for command_pair in commands:
      self._send_packet(
        self._get_packet_for_command(
          command_pair['command'],command_pair['data']))
      time.sleep(default_duration)

    self.stop()

  def do_moving_mixed_mode_test(self):
    #- Define test sequence
    command_pairs = [
      {'command':MIXED_BWD_FWD,'data':75},
      {'command':MIXED_LR_TURN,'data':64},
      {'command':MIXED_LR_TURN,'data':64+10},
      {'command':MIXED_LR_TURN,'data':64-15},
      {'command':MIXED_LR_TURN,'data':64},
      {'command':MIXED_LR_TURN,'data':64+11},
      {'command':MIXED_LR_TURN,'data':64},
      {'command':MIXED_BWD_FWD,'data':64-10},
      {'command':MIXED_BWD_FWD,'data':64-15},
      ]

    self.execute_timed_commands(command_pairs,2)
  
  def do_short_mixed_mode_test(self):
    #- Define test sequence
    command_pairs = [
      {'command':MIXED_BWD_FWD,'data':70},
      {'command':MIXED_LR_TURN,'data':64},
      {'command':MIXED_BWD_FWD,'data':75},
      {'command':MIXED_LR_TURN,'data':70},
      {'command':MIXED_LR_TURN,'data':75},
      {'command':MIXED_LR_TURN,'data':64},
      {'command':MIXED_LR_TURN,'data':60},
      ]

    self.execute_timed_commands(command_pairs,2)

  def do_short_test(self):
    try:
      # - Start moving right wheels forward, slowly
      self._send_packet(
        self._get_packet_for_command(
          M1_FWD,10))
      
      # - Don't change anything for a few seconds
      time.sleep(3)
      
      self._send_packet(
        self._get_packet_for_command(
          M1_BWD,10))
      
      # - Don't change anything for a few seconds
      time.sleep(3)

      self._send_packet(
        self._get_packet_for_command(
          M2_FWD,10))
      
      # - Don't change anything for a few seconds
      time.sleep(3)

      self._send_packet(
        self._get_packet_for_command(
          M2_BWD,10))
      
      # - Don't change anything for a few seconds
      time.sleep(3)
    finally:
      self._send_packet(
        self._get_packet_for_command(
          M1_FWD,0))
      self._send_packet(
        self._get_packet_for_command(
          M2_FWD,0))

  def test_M1_FWD_SLOW(self):
    try:
      # - Start moving right wheels forward, slowly
      self._send_packet(
        self._get_packet_for_command(
          M1_FWD,10))
      
      # - Don't change anything for a few seconds
      time.sleep(5)
    finally:
      self._send_packet(
        self._get_packet_for_command(
          M1_FWD,0))
  
  # ----------- End of Test Scripts ------------ 

class SabertoothPacketizedAdapterGPIO(SabertoothPacketizedAdapter):
  
  _gpio_tx_port = None
  _gpio_baud = None
  _gpio_offset = None
  _gpio_stop_bits = None
  _serial_address = None

  # - Create and setup the motor controllor adapter
  def __init__(self,
    port=DEFAULT_GPIO_TX,
    baudrate=DEFAULT_BAUD,
    serial_address=DEFAULT_ADDRESS,
    word_offset=DEFAULT_GPIO_WORD_OFFSET,
    stop_bits=DEFAULT_STOP_BITS):

    if SIM_MODE:
      print 'In sim mode.  Commands will not be sent to TXD.'

    # Create serial connection
    self._conn = pigpio.pi()
    # Initialize GPIO TX Pin
    self._conn.set_mode(port,pigpio.OUTPUT)
    self._conn.set_pull_up_down(port, pigpio.PUD_DOWN)

    # Define the settings required by pigpio api
    self._gpio_tx_port = port
    self._gpio_baud = baudrate
    self._gpio_offset = word_offset
    self._gpio_stop_bits = stop_bits

    # Initialize sabertooth
    time.sleep(3)
    self._write_bytes(bytearray([AUTO_BAUD_RATE_COMMAND]))

    self._serial_address = serial_address

    # Sabertooth requires you to define a min battery voltage upon startup
    self._send_packet(
      self._get_packet_for_command(
        MIN_VOLTAGE_COMMAND,10))

  def _shutdown(self):
    self._conn.stop()

  def _write_bytes(self,data_bytes):
    # write convention for pigpio (http://abyz.co.uk/rpi/pigpio/python.html)
    # 1. wave_clear()
    # 2. wave_add_*()
    # 3. id = wave_create()
    # 4. wave_send_*(id)

    # wait 1 ms before clearing, as a heuristic precaution against blowing away the previous waveform
    time.sleep(0.001)
    self._conn.wave_clear()

    self._conn.wave_add_serial(
      self._gpio_tx_port,
      self._gpio_baud,
      data_bytes)
      #self._gpio_offset,
      #8.0*len(data_bytes),
      #2.0*self._gpio_stop_bits)

    wf_id = self._conn.wave_create()
    if SIM_MODE:
      print "size of current waveform: {micros} us; {pulses} pulses".format(micros=self._conn.wave_get_micros(),pulses=self._conn.wave_get_pulses())
    else:
      self._conn.wave_send_once(wf_id)

      while self._conn.wave_tx_busy():
        time.sleep(0.01)

