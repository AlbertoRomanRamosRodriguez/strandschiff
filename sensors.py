from abc import ABC, abstractmethod
from time import sleep

import serial
import pynmea2
import smbus

class Sensor(ABC):

    @abstractmethod
    def measure(self):
        pass

    @property
    @abstractmethod
    def last_state(self) -> tuple:
        pass

class GPS(Sensor):
    """
    This class represents a GPS receiver connected through a serial port.

    Attributes:
        _NAME (str): Internal name for the GPS receiver.
        _PORT (str): Serial port the GPS is connected to.
        _BAUDRATE (int): Baud rate for serial communication.
        _TIMEOUT (int): Timeout value for serial communication in seconds.
        _serial (serial.Serial): Serial object for communication.
        _last_position (tuple): Last measured valid position (latitude, longitude).
    """

    def __init__(self, name: str, port: str, baudrate: int, timeout: int, wait_for_init=True) -> None:
        """
        Initializes a GPS object.

        Args:
            name (str): Name for the GPS receiver.
            port (str): Serial port the GPS is connected to (e.g., ttyAMA0 on Raspberry Pi).
            baudrate (int): Baud rate for serial communication (e.g., 9600).
            timeout (int): Timeout value for serial communication in seconds.
        """
        super().__init__()
        self._NAME = name
        self._PORT = port
        self._BAUDRATE = baudrate
        self._TIMEOUT = timeout

        self._serial = serial.Serial(self._PORT, baudrate=self._BAUDRATE)

        self._last_position = (0, 0)
        # Wait for 10 seconds to allow GPS to establish connection
        if wait_for_init:
            print(f"Waiting for the output of the {self._NAME} to be ready")
            sleep(120)
            print("Ready")

    def __repr__(self) -> str:
        """
        Returns a string representation of the GPS object.
        """
        return f"GPS(name='{self._NAME}', port='{self._PORT}', baud_rate={self._BAUDRATE}, timeout={self._TIMEOUT})"

    def measure(self, verbose: int = 0) -> tuple:
        """
        Measures the current GPS position and returns a tuple of (latitude, longitude).

        Args:
            verbose (int, optional): Verbosity level. If greater than 0, prints debug information. Defaults to 0.

        Returns:
            tuple: A tuple containing the measured latitude and longitude in degrees (rounded to 3 decimal places).
        """
        is_valid_coord = False

        if verbose > 0:
            print(f"Reading data from {self._NAME} at {self.PORT}")

        while not is_valid_coord:
            # Removed reference to pynmea2 for clarity as external library
            newdata = self._serial.readline()

            try:
                tag = newdata[0:6].decode('ascii')
            except Exception as e:
                if verbose > 0:
                    print(e)
                continue

            decoded_newdata = newdata.decode('ascii')

            if tag != "$GPRMC":
                continue

            newmsg=pynmea2.parse(decoded_newdata)
            lat = round(newmsg.latitude, 3)
            lng = round(newmsg.longitude, 3)

            self._last_position = (lat, lng)
            is_valid_coord = True

        return self._last_position

    @property
    def last_state(self) -> tuple:
        """
        Returns the last measured valid GPS position (latitude, longitude).
        """
        return self._last_position

    @property
    def NAME(self) -> str:
        """
        Returns the name assigned to the GPS receiver.
        """
        return self._NAME

    @property
    def BAUDRATE(self) -> int:
        """
        Returns the baud rate used for serial communication with the GPS.
        """
        return self._BAUDRATE

    @property
    def PORT(self) -> str:
        """
        Returns the serial port the GPS is connected to.
        """
        return self.PORT

    @property
    def TIMEOUT(self) -> int:
        """
        Returns the timeout value used for serial communication with the GPS (in seconds).
        """
        return self._TIMEOUT

class IMU(Sensor):

    def __init__(self, PWR_MGMT_1, SMPLRT_DIV, CONFIG, GYRO_CONFIG, INT_ENABLE, DEV_ADDR, bus=1) -> None:
        """
        Initializes an IMU object.

        Args:
            PWR_MGMT_1 (int): Register address for power management 1.
            SMPLRT_DIV (int): Register address for sample rate divider.
            CONFIG (int): Register address for configuration.
            GYRO_CONFIG (int): Register address for gyroscope configuration.
            INT_ENABLE (int): Register address for interrupt enable.
            DATA_REGISTERS (dict): Dictionary mapping register names to their addresses.
            DEV_ADDR (int): I2C device address.
            bus (int, optional): I2C bus number. Defaults to 1.
        """
        super().__init__()

        self._PWR_MGMT_1 = PWR_MGMT_1
        self._SMPLRT_DIV = SMPLRT_DIV
        self._CONFIG = CONFIG
        self._GYRO_CONFIG = GYRO_CONFIG
        self._INT_ENABLE = INT_ENABLE
        self._bus = smbus.SMBus(bus)  # Initialize bus outside to avoid modifying arguments
        self._DEV_ADDR = DEV_ADDR
        self._registers = {
            'ACCEL_XOUT' : 0x3B,
            'ACCEL_YOUT' : 0x3D,
            'ACCEL_ZOUT' : 0x3F,
            'GYRO_XOUT' : 0x43,
            'GYRO_YOUT' : 0x45,
            'GYRO_ZOUT' : 0x47
            
            }
        self._calibrated_values = {}

        # write to sample rate regiser
        self._bus.write_byte_data(DEV_ADDR, SMPLRT_DIV, 7)
        
        # write to power management regiser
        self._bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 1)
        
        # write to Configuration regiser
        self._bus.write_byte_data(DEV_ADDR, CONFIG, 0)
        
        # write to Gyro Configuration regiser
        self._bus.write_byte_data(DEV_ADDR, GYRO_CONFIG, 24)
        
        # write to Interrupt enable regiser
        self._bus.write_byte_data(DEV_ADDR, INT_ENABLE, 1)
    
        self._last_state_dict = {}


    def _read_raw_data(self,addr):
        """
        Reads raw data from a specific register address.

        Args:
            addr (int): Register address to read from.

        Returns:
            int: 16-bit signed integer value from the register.
        """
        # Accel and gyro data are 16-bit
        
        high = self._bus.read_byte_data(self._DEV_ADDR, addr)
        low = self._bus.read_byte_data(self._DEV_ADDR, addr+1)
        
        # concatenate higher and lwoer value
        value = ((high << 8) | low)
        
        # to get signed value from MPU6050
        value = value - 65536 if value > 32768 else value
        
        
        return value
        
    def measure(self) -> tuple:
        """
        Measures sensor data and returns a tuple of scaled values.

        Returns:
            tuple: Tuple containing accelerometer and gyroscope readings.
        """
        measured_values = {reg:self._read_raw_data(addr) for reg, addr in self._registers.items()}
        for i, pair in enumerate(measured_values.items()):
            reg, value = pair
            
            if i < 3:
               measured_values[reg] = round(value / 16384.0, 3)
            else:
               measured_values[reg] = round(value / 131.0, 3)

        self._last_state_dict = measured_values

        return tuple(measured_values.values())
    
    @property
    def last_state(self) -> tuple:
        """
        Returns the last measured sensor data as a tuple.

        Returns:
            tuple: Tuple containing the last measured accelerometer and gyroscope readings.
        """
        return self._last_state_dict

    def __repr__(self):
        """
        Returns a string representation of the IMU object.
        """
        return ""
        

