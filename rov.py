import RPi.GPIO as GPIO
from sensors import GPS,IMU

class ROV:
  """
  Class representing a Remotely Operated Vehicle (ROV).
  """
  def __init__(self, pin_configs):
    """
    Initializes the ROV with a list of dictionaries containing pin configurations for the IMU, GPS, and motors.

    Args:
      pin_configs (list of dict): A list of dictionaries containing pin configurations.
          - Each dictionary should have keys for sensor/motor names (e.g., "imu", "gps", "motor1", "motor2") and
            corresponding GPIO pin numbers as values.
    """
    self._imu = None
    self._gps = None
    self._motor1 = None
    self._motor2 = None

    # Initialize IMU, GPS, and motor objects based on pin configurations
    for config in pin_configs:
      if config["name"] == "imu":
        name, port, baudrate, timeout, wait_for_init = tuple(config["pins"].values())

        self._imu = IMU(name,port,baudrate,timeout,wait_for_init)

      elif config["name"] == "gps":
        PWR_MGMT_1, SMPLRT_DIV, CONFIG, GYRO_CONFIG, INT_ENABLE, DEV_ADDR, bus= tuple(config["pins"].values())

        self._gps = GPS(PWR_MGMT_1, SMPLRT_DIV, CONFIG, GYRO_CONFIG, INT_ENABLE, DEV_ADDR, bus)

      elif config["name"] == "motor1":
        self._motor1 = Motor(config["pins"])
      elif config["name"] == "motor2":
        self._motor2 = Motor(config["pins"])
      else:
        print(f"WARNING: Unknown component name: {config['name']}")

    # Check if all required components were initialized
    if self._imu is None:
      print("Error: IMU not found in pin configurations.")
    if self._gps is None:
      print("Error: GPS not found in pin configurations.")
    if self._motor1 is None or self._motor2 is None:
      print("Error: Motors not found in pin configurations.")

  def start(self):
    """
    Starts the ROV (e.g., initializes sensors, prepares motors).
    """
    # Implement logic to start sensors (e.g., enabling I2C for IMU) and motors (e.g., setting initial duty cycle to 0)
    if self._imu:
      # Add code to start IMU (e.g., enabling I2C communication)
      pass
    if self._gps:
      # Add code to start GPS (e.g., enabling serial communication)
      pass
    self._motor1.stop()  # Ensure motors are stopped initially
    self._motor2.stop()

  def stop(self):
    """
    Stops the ROV (e.g., cleans up sensors, stops motors).
    """
    # Implement logic to stop sensors and motors
    if self._imu:
      # Add code to stop IMU (e.g., disabling I2C communication)
      pass
    if self._gps:
      # Add code to stop GPS (e.g., disabling serial communication)
      pass
    self._motor1.stop()
    self._motor2.stop()

  def read_sensors(self):
    """
    Reads data from the IMU and GPS sensors.

    Returns:
      tuple: A tuple containing an IMU data tuple (timestamp, acceleration, gyroscope) and a GPS data tuple (timestamp, latitude, longitude), or None if sensors are not available.
    """
    imu_data = None
    gps_data = None
    if self._imu:
      imu_data = self._imu.measure()
    if self._gps:
      gps_data = self._gps.measure()
    return imu_data, gps_data

  def control_motors(self, duty_cycle1, duty_cycle2):
    """
    Controls the speed of the two motors using PWM duty cycles (0-100).

    Args:
      duty_cycle1 (int): The desired duty cycle for motor 1 (0-100).
      duty_cycle2 (int): The desired duty cycle for motor 2 (0-100).
    """
    if self._motor1:
      self._motor1.set_duty_cycle(duty_cycle1)
    if self._motor2:
      self._motor2.set_duty_cycle(duty_cycle2)
