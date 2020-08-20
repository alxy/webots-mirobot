import os
import time

import zmq

from .serial_device import SerialDevice
from .fake_device import FakeDevice
from .exceptions import MirobotError, MirobotAlarm, MirobotReset, MirobotAmbiguousPort


class IpInterface:
    """ A class for bridging the interface between `mirobot.base_mirobot.BaseMirobot` and `mirobot.serial_device.SerialDevice`"""
    def __init__(self, mirobot, host='localhost', port=5555, stopbits=None, exclusive=True, debug=False, logger=None):
        """ Initialization of `IpInterface` class

        Parameters
        ----------
        mirobot : `mirobot.base_mirobot.BaseMirobot`
            Mirobot object that this instance is attached to.
        portname : str
             (Default value = None) The portname to attach to. If `None`, and the `autofindport` parameter is `True`, then this class will automatically try to find an open port. It will attach to the first one that is available.
        baudrate : int
             (Default value = None) Baud rate of the connection.
        stopbits : int
             (Default value = None) Stopbits of the connection.
        exclusive : bool
             (Default value = True) Whether to exclusively block the port for this instance. Is only a true toggle on Linux and OSx; Windows always exclusively blocks serial ports. Setting this variable to `False` on Windows will throw an error.
        debug : bool
             (Default value = False) Whether to show debug statements in logger.
        logger : logger.Logger
             (Default value = None) Logger instance to use for this class. Usually `mirobot.base_mirobot.BaseMirobot.logger`.
        autofindport : bool
             (Default value = True) Whether to automatically search for an available port if `address` parameter is `None`.

        Returns
        -------

        """

        self.mirobot = mirobot
        self.host = host
        self.port = port
        self._is_connected = False

        if logger is not None:
            self.logger = logger

        self._debug = debug
        # serial_device_kwargs = {'debug': debug, 'exclusive': exclusive}

        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)

    @property
    def debug(self):
        """ Return the `debug` property of `SerialInterface` """
        return self._debug

    @debug.setter
    def debug(self, value):
        """
        Set the new value for the `debug` property of `mirobot.serial_interface.SerialInterface`. Use as in `BaseMirobot.setDebug(value)`.
        Use this setter method as it will also update the logging objects of `mirobot.serial_interface.SerialInterface` and its `mirobot.serial_device.SerialDevice`. As opposed to setting `mirobot.serial_interface.SerialInterface._debug` directly which will not update the loggers.

        Parameters
        ----------
        value : bool
            The new value for `mirobot.serial_interface.SerialInterface._debug`.

        """
        self._debug = bool(value)

    def send(self, msg, disable_debug=False, terminator=os.linesep, wait=True, wait_idle=True):
        """
        Send a message to the Mirobot.

        Parameters
        ----------
        msg : str or bytes
             A message or instruction to send to the Mirobot.
        var_command : bool
            (Default value = `False`) Whether `msg` is a variable command (of form `$num=value`). Will throw an error if does not validate correctly.
        disable_debug : bool
            (Default value = `False`) Whether to override the class debug setting. Used primarily by ` BaseMirobot.device.wait_until_idle`.
        terminator : str
            (Default value = `os.linesep`) The line separator to use when signaling a new line. Usually `'\\r\\n'` for windows and `'\\n'` for modern operating systems.
        wait : bool
            (Default value = `None`) Whether to wait for output to end and to return that output. If `None`, use class default `BaseMirobot.wait` instead.
        wait_idle : bool
            (Default value = `False`) Whether to wait for Mirobot to be idle before returning.

        Returns
        -------
        msg : List[str] or bool
            If `wait` is `True`, then return a list of strings which contains message output.
            If `wait` is `False`, then return whether sending the message succeeded.
        """

        output = self.socket.send_string(msg)

        if self._debug and not disable_debug:
            self.logger.debug(f"[SENT] {msg}")

        if wait:
            output = self.wait_for_ok(disable_debug=disable_debug)

            # if wait_idle:
            #     self.wait_until_idle()

        return output

    @property
    def is_connected(self):
        """
        Check if Mirobot is connected.

        Returns
        -------
        connected : bool
            Whether the Mirobot is connected.
        """
        return self._is_connected

    def wait_for_ok(self, reset_expected=False, disable_debug=False):
        """
        Continuously loops over and collects message output from the serial device.
        It stops when it encounters an 'ok' or otherwise terminal condition phrase.

        Parameters
        ----------
        reset_expected : bool
            (Default value = `False`) Whether a reset string is expected in the output (Example: on starting up Mirobot, output ends with a `'Using reset pos!'` rather than the traditional `'Ok'`)
        disable_debug : bool
            (Default value = `False`) Whether to override the class debug setting. Otherwise one will see status message debug output every 0.1 seconds, thereby cluttering standard output. Used primarily by `BaseMirobot.wait_until_idle`.

        Returns
        -------
        output : List[str]
            A list of output strings upto and including the terminal string.
        """
        #  Do some 'work'
        # time.sleep(1)
        # return ['test']
        output = ['']

        ok_eols = ['ok']

        reset_strings = ['Using reset pos!']

        def matches_eol_strings(terms, s):
            for eol in terms:
                if s.endswith(eol):
                    return True
            return False

        if reset_expected:
            eols = ok_eols + reset_strings
        else:
            eols = ok_eols

        eol_threshold = 1
        eol_counter = 0
        while eol_counter < eol_threshold:
            msg = self.socket.recv_string()

            if self._debug and not disable_debug and msg:
                self.logger.debug(f"[RECV] {msg}")

            if 'error' in msg:
                self.logger.error(MirobotError(msg.replace('error: ', '')))

            if 'ALARM' in msg:
                self.logger.error(MirobotAlarm(msg.split('ALARM: ', 1)[1]))

            output.append(msg)

            if not reset_expected and matches_eol_strings(reset_strings, msg):
                self.logger.error(MirobotReset('Mirobot was unexpectedly reset!'))

            if matches_eol_strings(eols, output[-1]):
                eol_counter += 5

        return output[1:]  # don't include the dummy empty string at first index

    def wait_until_idle(self, refresh_rate=0.1):
        """
        Continuously loops over and refreshes state of the Mirobot.
        It stops when it encounters an 'Idle' state string.

        Parameters
        ----------
        refresh_rate : float
            (Default value = `0.1`) The rate in seconds to check for the 'Idle' state. Choosing a low number might overwhelm the controller on Mirobot. Be cautious when lowering this parameter.

        Returns
        -------
        output : List[str]
            A list of output strings upto and including the terminal string.
        """
        self.mirobot.update_status(disable_debug=True)

        while self.mirobot.status.state != 'Idle':
            time.sleep(refresh_rate)
            self.mirobot.update_status(disable_debug=True)

    def connect(self, portname=None):
        """
        Connect to the Mirobot.

        Parameters
        ----------
        portname : str
            (Default value = `None`) The name of the port to connnect to. If this is `None`, then it will try to use `self.default_portname`. If both are `None`, then an error will be thrown. To avoid this, specify a portname.

        Returns
        -------
        ok_msg : List[str]
            The output from an initial Mirobot connection.
        """
        self.socket.connect("tcp://{}:{}".format(self.host, self.port))
        self._is_connected = True

    def disconnect(self):
        """ Disconnect from the Mirobot. Close the serial device connection. """
        # self.socket.disconnect("tcp://{}:{}".format(self.host, self.port))
        self._is_connected = False
