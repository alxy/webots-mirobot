try:
    from ikpy.chain import Chain
    from ikpy.link import OriginLink, URDFLink
except ImportError:
    import sys
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')

import math
import numpy as np
from controller import Supervisor
import tempfile
from ip_controller import IpController
#from controller import Robot

import zmq
import re

# Initialize the Webots Supervisor.
supervisor = Supervisor()
timeStep = int(4 * supervisor.getBasicTimeStep())

# Create the arm chain from the URDF
# filename = None
# with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
#     filename = file.name
#     file.write(supervisor.getUrdf().encode('utf-8'))
    
# armChain = Chain.from_urdf_file(filename)

# Initialize the arm motors.
# motors = []
# for motorName in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']:
#     motor = supervisor.getPositions(motorName)
#     motor.setVelocity(1.0)
#     motors.append(motor)

# Get the arm and target nodes.
# target = supervisor.getFromDef('TARGET')
# mirobot = supervisor.getSelf()
# effector = supervisor.getFromDef('EFFECTOR')
# robot = Robot()  
# 
gripper = supervisor.getFromDef('GRIPPER')
# Start the zmq server
# 
# context = zmq.Context()
# socket = context.socket(zmq.REP)
# socket.bind("tcp://*:5555")
# print('Server started...')

controller = IpController(supervisor, timestep=timeStep)  
controller.initialize_motors()
controller.initialize_sensors()

# armChain = controller.armChain
# motors = list(controller.motors.values())
# print(motors)
# done = False

translation = [0.2, 0.2, 0.1]

# first = [0.27, 0, 0.030]
# second = [0.200, 0, 0.150]
# controller.open_gripper()
# controller.go_to_cartesian_lin(first)
# controller.close_gripper()
# controller.go_to_cartesian_lin(second)

while supervisor.step(timeStep) != -1:
    t = supervisor.getTime()

    controller.run()
    
    # print(gripper.getPosition())

    # try:
    #     #  Wait for next request from client
    #     gcode = socket.recv_string(flags=zmq.NOBLOCK)
    #     print("Received request: %s" % gcode)

    #     #  parse the gcode and do something
    #     if gcode.startswith('$HH'):
    #         print('homing')
            

    #     elif gcode.startswith('M21 G90'):
    #         print('angle mode motion')
    #         # M21 G90 X{1} Y{2} Z{3} A{4} B{5} C{6} F{7}

    #     elif gcode.startswith('M20 G90 G1'):
    #         print('cartesian linear motion')
    #         # M20 G90 G1 X{1} Y{2} Z{3} A{4} B{5} C{6} F{7}
    #         regex = r"M20 G90 G1 X(?P<x>[\d\.-]+) Y(?P<y>[\d\.-]+) Z(?P<z>[\d\.-]+) A(?P<a>[\d\.-]+) B(?P<b>[\d\.-]+) C(?P<c>[\d\.-]+) F(?P<f>[\d\.-]+)"

    #         matches = re.match(regex, gcode)
    #         params = matches.groupdict()
    #         print(params)

    #         translation = np.array([float(params['x']), float(params['y']), float(params['z'])]) / 1000 # Convert mm -> m
    #         orientation = np.array([float(params['a']), float(params['b']), float(params['c'])])
    #         speed = float(params['f']) / 2000

    #         socket.send_string("ok")

    #     else:
    #         #  Send reply back to client
    #         socket.send_string("ok")
    # except zmq.Again as e:
    #     print("No message received yet")

    # if t < 1:
    #     # Call "ikpy" to compute the inverse kinematics of the arm.
    #     ikResults = armChain.inverse_kinematics(translation)
    #     #print(ikResults)

    #     # Actuate the 3 first arm motors with the IK results.
    #     for i in range(len(motors)):
    #         motors[i].setPosition(ikResults[i + 1])
        
    # else:
    #     if not done:
    #         # controller.parse_gcode('$HH')
    #         controller.go_to_cartesian_lin([-0.2, 0.1, 0.1])
    #         done = True

                
    #if (t % 10*timeStep) == 0:
    #print(effector.getPosition())       
    # motors[0].setVelocity(leftSpeed)