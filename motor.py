import RPi.GPIO as GPIO

class Motor:
  """
  Class representing a DC motor controlled by a PWM signal using RPi.GPIO library.
  """
  def __init__(self, pins, duty_cycle=0):
    """
    Initializes the motor with a dictionary of pin names as keys and their corresponding GPIO pin numbers as values, and an initial duty cycle (0-100).

    Args:
      pins (dict): Dictionary containing pin names as keys and their corresponding GPIO pin numbers as values.
      duty_cycle (int, optional): Initial duty cycle for the motor (0-100). Defaults to 0.
    """
    self.pins = pins
    self.duty_cycle = duty_cycle
    GPIO.setmode(GPIO.BCM)  # Set GPIO pin numbering mode
    GPIO.setup(self.pins["IN1"], GPIO.OUT)  # Set IN1 pin as output
    GPIO.setup(self.pins["IN2"], GPIO.OUT)  # Set IN2 pin as output
    GPIO.setup(self.pins["ENA"], GPIO.OUT)  # Set ENA pin (PWM) as output

  def set_duty_cycle(self, duty_cycle):
    """
    Sets the motor's PWM duty cycle (0-100).

    Args:
      duty_cycle (int): The desired duty cycle for the motor (0-100).
    """
    if 0 <= duty_cycle <= 100:
      self.duty_cycle = duty_cycle
      pwm = GPIO.PWM(self.pins["ENA"], 1000)  # Create PWM instance with 1 kHz frequency
      pwm.start(duty_cycle)  # Start PWM with the specified duty cycle
    else:
      raise ValueError("Duty cycle must be between 0 and 100.")

  def forward(self):
    """
    Starts the motor in the forward direction.
    """
    GPIO.output(self.pins["IN1"], GPIO.HIGH)  # Set IN1 high for forward
    GPIO.output(self.pins["IN2"], GPIO.LOW)  # Set IN2 low for forward
    self.set_duty_cycle(self.duty_cycle)  # Set the desired duty cycle

  def backward(self):
    """
    Starts the motor in the backward direction.
    """
    GPIO.output(self.pins["IN1"], GPIO.LOW)  # Set IN1 low for backward
    GPIO.output(self.pins["IN2"], GPIO.HIGH)  # Set IN2 high for backward
    self.set_duty_cycle(self.duty_cycle)  # Set the desired duty cycle

  def stop(self):
    """
    Stops the motor.
    """
    GPIO.output(self.pins["IN1"], GPIO.LOW)  # Set both IN pins low to stop
    GPIO.output(self.pins["IN2"], GPIO.LOW)
    GPIO.PWM(self.pins["ENA"], 0).stop  # Stop the PWM signal on ENA pin
