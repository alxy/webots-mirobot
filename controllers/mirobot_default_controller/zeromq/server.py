#
#   Hello World server in Python
#   Binds REP socket to tcp://*:5555
#   Expects b"Hello" from client, replies with b"World"
#

import time
import zmq
import re
import numpy as np

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

print("Server running...")

while True:
    try:
        #  Wait for next request from client
        gcode = socket.recv_string(flags=zmq.NOBLOCK)
        print("Received request: %s" % gcode)

        #  Do some 'work'
        if gcode.startswith('$HH'):
            print('homing')
            

        elif gcode.startswith('M21 G90'):
            print('angle mode motion')
            # M21 G90 X{1} Y{2} Z{3} A{4} B{5} C{6} F{7}

        elif gcode.startswith('M20 G90 G1'):
            print('cartesian linear motion')
            # M20 G90 G1 X{1} Y{2} Z{3} A{4} B{5} C{6} F{7}
            regex = r"M20 G90 G1 X(?P<x>[\d\.-]+) Y(?P<y>[\d\.-]+) Z(?P<z>[\d\.-]+) A(?P<a>[\d\.-]+) B(?P<b>[\d\.-]+) C(?P<c>[\d\.-]+) F(?P<f>[\d\.-]+)"

            print(gcode)

            matches = re.match(regex, gcode)
            params = matches.groupdict()
            print(params)

            translation = np.array([float(params['x']), float(params['y']), float(params['z'])]) / 1000 # Convert mm -> m
            orientation = np.array([float(params['a']), float(params['b']), float(params['c'])])
            speed = float(params['f']) / 2000

            socket.send_string("ok")

        else:
            #  Send reply back to client
            socket.send_string("ok")
    except zmq.Again as e:
        print("No message received yet")

    # perform other important stuff
    time.sleep(5)

    