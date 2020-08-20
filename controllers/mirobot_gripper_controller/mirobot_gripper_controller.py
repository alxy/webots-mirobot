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

#from controller import Robot


#mirobotChain = Chain.from_urdf_file("mirobot_urdf_2.xacro")
#print(mirobotChain)

# Initialize the Webots Supervisor.
supervisor = Supervisor()
timeStep = int(4 * supervisor.getBasicTimeStep())

# Create the arm chain from the URDF
filename = None
with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
    filename = file.name
    file.write(supervisor.getUrdf().encode('utf-8'))
    
armChain = Chain.from_urdf_file(filename)

# Get the arm and target nodes.
# target = supervisor.getFromDef('TARGET')
mirobot = supervisor.getSelf()
gripper = supervisor.getFromDef('GRIPPER')

# Initialize the arm motors.
motors = []
for motorName in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']:
    motor = supervisor.getMotor(motorName)
    motor.setVelocity(1.0)
    motors.append(motor)
    
motorsGripper = []
#for motorName in ['finger_joint', 'left_inner_finger_joint', 'left_inner_knuckle_joint', 'right_outer_knuckle_joint', 'right_inner_finger_joint', 'right_inner_knuckle_joint']:
for motorName in ['finger_joint','right_outer_knuckle_joint']:
    motor = supervisor.getMotor(motorName)
    motor.setVelocity(1.0)
    motorsGripper.append(motor)
    
sensor = supervisor.getPositionSensor('joint3_sensor')
sensor.enable(timeStep)


# robot = Robot()    


while supervisor.step(timeStep) != -1:
    t = supervisor.getTime()
    
    print(sensor.getValue())
    
    #motorsGripper[0].setPosition(0.35)
    #motorsGripper[1].setPosition(0.35)

    # Use the circle equation relatively to the arm base as an input of the IK algorithm.
    #x = 0.25 * math.cos(t) + 1.1
    #y = 0.25 * math.sin(t) - 0.95
    #z = 0.23
    if t > 3:
        motorsGripper[0].setPosition(0.4)
        motorsGripper[1].setPosition(0.4)
    else:
        motorsGripper[0].setPosition(0)
        motorsGripper[1].setPosition(0)
    
    x = 0.27 #-0.3
    y = 0.0
    z = 0.03
    
    if t > 4:
        x = 0.2 #-0.3
        y = 0.0
        z = 0.15
    
    coords_trans = [x, y, z]

    # Call "ikpy" to compute the inverse kinematics of the arm.
    ikResults = armChain.inverse_kinematics(coords_trans)
    #print(ikResults)

    # Actuate the 3 first arm motors with the IK results.
    for i in range(len(motors)):
        motors[i].setPosition(ikResults[i + 1])
    # Keep the hand orientation down.
    #motors[4].setPosition(-ikResults[2] - ikResults[3] + math.pi / 2)
    # Keep the hand orientation perpendicular.
    #motors[5].setPosition(ikResults[1])
                
    #if (t % 10*timeStep) == 0:
    #print(effector.getPosition())       
    # motors[0].setVelocity(leftSpeed)