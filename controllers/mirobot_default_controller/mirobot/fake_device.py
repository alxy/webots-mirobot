import logging
import os

import serial

from .exceptions import ExitOnExceptionStreamHandler, SerialDeviceOpenError, SerialDeviceReadError, SerialDeviceCloseError, SerialDeviceWriteError

class Serial:

    ## init(): the constructor.  Many of the arguments have default values
    # and can be skipped when calling the constructor.
    def __init__( self, port='COM1', baudrate = 19200, timeout=1,
                  bytesize = 8, parity = 'N', stopbits = 1, xonxoff=0,
                  rtscts = 0):
        self.name     = port
        self.port     = port
        self.timeout  = timeout
        self.parity   = parity
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.stopbits = stopbits
        self.xonxoff  = xonxoff
        self.rtscts   = rtscts
        self.is_open = True
        self._receivedData = ""
        self._data = "ok\r\n"
        self.test = 1

        # print(self._data)

    ## isOpen()
    # returns True if the port to the Arduino is open.  False otherwise
    def isOpen( self ):
        return self.is_open

    ## open()
    # opens the port
    def open( self ):
        self.is_open = True

    ## close()
    # closes the port
    def close( self ):
        self.is_open = False

    ## write()
    # writes a string of characters to the Arduino
    def write( self, string ):
        # print( 'Serial got: "' + string + '"' )
        self._receivedData += string.decode()
        print(self._receivedData)

    ## read()
    # reads n characters from the fake Arduino. Actually n characters
    # are read from the string _data and returned to the caller.
    def read( self, n=1 ):
        s = self._data[0:n]
        self._data = self._data[n:]
        #print( "read: now self._data = ", self._data )
        return s

    ## readline()
    # reads characters from the fake Arduino until a \n is found.
    def readline( self, terminator=os.linesep ):
        try:
            returnIndex = self._data.index(terminator)
            s = self._data[0:returnIndex+1]
            self._data = self._data[returnIndex+1:]
            return s
        except ValueError:
            if self.test % 3  != 0:
                self.test = self.test+1
                return "ok"
            else:
                return "<Idle,Angle(ABCDXYZ):1,2,3,4,5,6,7,Cartesian coordinate(XYZRxRyRz):1,2,3,4,5,6,Pump PWM:1,Value PWM:2,Motion_MODE:0>"
            
            


class FakeDevice:
    """ A class for establishing a connection to a serial device. """
    def __init__(self, portname='', baudrate=0, stopbits=1, exclusive=True, debug=False):
        """ Initialization of `SerialDevice` class

        Parameters
        ----------
        portname : str
             (Default value = `''`) Name of the port to connect to. (Example: 'COM3' or '/dev/ttyUSB1')
        baudrate : int
             (Default value = `0`) Baud rate of the connection.
        stopbits : int
             (Default value = `1`) Stopbits of the connection.
        exclusive : bool
             (Default value = `True`) Whether to (try) forcing exclusivity of serial port for this instance. Is only a true toggle on Linux and OSx; Windows always exclusively blocks serial ports. Setting this variable to `False` on Windows will throw an error.
        debug : bool
             (Default value = `False`) Whether to print DEBUG-level information from the runtime of this class. Show more detailed information on screen output.

        Returns
        -------
        class : SerialDevice

        """
        self.portname = str(portname)
        self.baudrate = int(baudrate)
        self.stopbits = int(stopbits)
        self.exclusive = exclusive
        self._debug = debug

        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)

        self.stream_handler = ExitOnExceptionStreamHandler()
        self.stream_handler.setLevel(logging.DEBUG if self._debug else logging.INFO)

        formatter = logging.Formatter(f"[{self.portname}] [%(levelname)s] %(message)s")
        self.stream_handler.setFormatter(formatter)
        self.logger.addHandler(self.stream_handler)

        self.serialport = Serial()
        self._is_open = False


    def __del__(self):
        """ Close the serial port when the class is deleted """
        self.close()

    @property
    def debug(self):
        """ Return the `debug` property of `SerialDevice` """
        return self._debug

    @debug.setter
    def debug(self, value):
        """
        Set the new `debug` property of `SerialDevice`. Use as in `SerialDevice.setDebug(value)`.

        Parameters
        ----------
        value : bool
            The new value for `SerialDevice.debug`. User this setter method as it will also update the logging method. As opposed to setting `SerialDevice.debug` directly which will not update the logger.

        """
        self._debug = bool(value)
        self.stream_handler.setLevel(logging.DEBUG if self._debug else logging.INFO)

    @property
    def is_open(self):
        """ Check if the serial port is open """
        self._is_open = self.serialport.is_open
        return self._is_open

    def listen_to_device(self):
        """
        Listen to the serial port and return a message.

        Returns
        -------
        msg : str
            A single line that is read from the serial port.

        """
        while self._is_open:
            try:
                msg = self.serialport.readline()
                if msg != b'':
                    msg = msg.strip()
                    return msg

            except Exception as e:
                self.logger.exception(SerialDeviceReadError(e))

    def open(self):
        """ Open the serial port. """
        if not self._is_open:
            # serialport = 'portname', baudrate, bytesize = 8, parity = 'N', stopbits = 1, timeout = None, xonxoff = 0, rtscts = 0)
            self.serialport.port = self.portname
            self.serialport.baudrate = self.baudrate
            self.serialport.stopbits = self.stopbits

            try:
                self.logger.debug(f"Attempting to open serial port {self.portname}")

                self.serialport.open()
                self._is_open = True

                self.logger.debug(f"Succeeded in opening serial port {self.portname}")

            except Exception as e:
                self.logger.exception(SerialDeviceOpenError(e))

    def close(self):
        """ Close the serial port. """
        if self._is_open:
            try:
                self.logger.debug(f"Attempting to close serial port {self.portname}")

                self._is_open = False
                self.serialport.close()

                self.logger.debug(f"Succeeded in closing serial port {self.portname}")

            except Exception as e:
                self.logger.exception(SerialDeviceCloseError(e))

    def send(self, message, terminator=os.linesep):
        """
        Send a message to the serial port.


        Parameters
        ----------
        message : str
            The string to send to serial port.

        terminator : str
            (Default value = `os.linesep`) The line separator to use when signaling a new line. Usually `'\\r\\n'` for windows and `'\\n'` for modern operating systems.

        Returns
        -------
        result : bool
            Whether the sending of `message` succeeded.

        """
        print(message)
        if self._is_open:
            try:
                if not message.endswith(terminator):
                    message += terminator
                self.serialport.write(message.encode('utf-8'))

            except Exception as e:
                self.logger.exception(SerialDeviceWriteError(e))

            else:
                return True
        else:
            return False
