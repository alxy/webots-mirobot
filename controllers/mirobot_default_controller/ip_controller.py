import zmq
from ikpy.chain import Chain
import tempfile
import queue
import re
import numpy as np
from ik_module import inverseKinematics
from ikpy.urdf.URDF import get_urdf_parameters

class IpController:

    motorNames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'finger_joint','right_outer_knuckle_joint']
    sensorNames = ['joint1_sensor', 'joint2_sensor', 'joint3_sensor', 'joint4_sensor', 'joint5_sensor', 'joint6_sensor', 'finger_joint_sensor','right_outer_knuckle_joint_sensor']

    jointPositions = []

    motors = {}
    sensors = {}

    def __init__(self, supervisor, timestep=32, host='*', port=5555):
        self.supervisor = supervisor
        self.timestep = 32
        self.host = host
        self.port = port

        self.armChain = self.create_arm_chain()
        # self.armChain.links = self.armChain.links[:7]
        print(self.armChain)
        self.gripper = supervisor.getFromDef('GRIPPER')
        # self.ik = inverseKinematics(supervisor)

        self.actions = queue.Queue()

        self.bind()
        self.should_send_response = False

    def bind(self):
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.bind("tcp://{}:{}".format(self.host, self.port))

    def read(self):
        try:
            #  Wait for next request from client
            gcode = self.socket.recv_string(flags=zmq.NOBLOCK)
            print("Received request: %s" % gcode)
            return gcode
        except zmq.Again as e:
            return None

    def initialize_motors(self):
        for motorName in self.motorNames:
            motor = self.supervisor.getMotor(motorName)
            # motor.setVelocity(1.0)
            self.motors[motorName] = motor

    def initialize_sensors(self):
        for sensorName in self.sensorNames:
            sensor = self.supervisor.getPositionSensor(sensorName)
            sensor.enable(self.timestep)
            self.sensors[sensorName] = sensor

    def create_arm_chain(self):
        filename = None
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
            filename = file.name
            file.write(self.supervisor.getUrdf().encode('utf-8'))
            
        armChain = Chain.from_urdf_file(filename)
        armChain = Chain(armChain.links[:7])
        print(armChain)

        return armChain

    def get_joint_positions(self):
        positions = []
        for sensor in self.sensors.values():
            positions.append(sensor.getValue())
        
        return positions

    def get_gripper_position(self):
        position_world = self.gripper.getPosition()

        return np.array([position_world[0], position_world[2], position_world[1]])

    def is_busy(self, new_positions):
        # new_positions = self.get_joint_positions()
        old_positions = self.jointPositions

        for new, old in zip(new_positions, old_positions):
            if round(new, 4) != round(old, 4): # numeric calculations, values are not exactly equal
                return True

        return False

    def run(self):
        positions = self.get_joint_positions()
        # print(positions)

        if not self.is_busy(positions):
            if not self.actions.empty():
                commands = self.actions.get()

                for motorName, position, speed in commands:
                    motor = self.motors[motorName]
                    motor.setPosition(position)
                    motor.setVelocity(0.3 * speed * motor.getMaxVelocity())
            else:
                if self.should_send_response:
                    self.send_response("ok")

                gcode = self.read()
                if gcode:
                    self.parse_gcode(gcode)
                    self.should_send_response = True

        else:
            # print('Moving...')
            pass

        self.jointPositions = positions # Update joint positions

    def send_response(self, msg):
        self.socket.send_string(msg)
        self.should_send_response = False

    def parse_gcode(self, gcode):
        if gcode.startswith('$HH'):
            print('homing')
            self.home_individual()
            

        elif gcode.startswith('M21 G90'):
            print('angle mode motion')
            # M21 G90 X{1} Y{2} Z{3} A{4} B{5} C{6} F{7}

        elif gcode.startswith('M20 G90 G1'):
            print('cartesian linear motion')
            # M20 G90 G1 X{1} Y{2} Z{3} A{4} B{5} C{6} F{7}
            regex = r"M20 G90 G1 X(?P<x>[\d\.-]+) Y(?P<y>[\d\.-]+) Z(?P<z>[\d\.-]+) A(?P<a>[\d\.-]+) B(?P<b>[\d\.-]+) C(?P<c>[\d\.-]+) F(?P<f>[\d\.-]+)"

            matches = re.match(regex, gcode)
            params = matches.groupdict()

            translation = np.array([float(params['x']), float(params['y']), float(params['z'])]) / 1000 # Convert mm -> m
            orientation = np.array([float(params['a']), float(params['b']), float(params['c'])])
            speed = float(params['f']) / 2000

            self.go_to_cartesian_lin(translation, orientation, speed)

        elif gcode.startswith('M20 G91 G1'):
            print('cartesian increment motion')
            # M20 G91 G1 X{x} Y{y} Z{z} A{a} B{b} C{c} F{speed}
            regex = r"M20 G91 G1( X(?P<x>[\d\.-]+))?( Y(?P<y>[\d\.-]+))?( Z(?P<z>[\d\.-]+))?( A(?P<a>[\d\.-]+))?( B(?P<b>[\d\.-]+))?( C(?P<c>[\d\.-]+))?( F(?P<f>[\d\.-]+))?"

            matches = re.match(regex, gcode)
            params = matches.groupdict()

            translation = np.array([float(params.get('x') or 0), float(params.get('y') or 0), float(params.get('z') or 0)]) / 1000 # Convert mm -> m
            orientation = np.array([float(params.get('a') or 0), float(params.get('b') or 0), float(params.get('c') or 0)])
            speed = float(params.get('f') or 0) / 2000

            self.increment_cartesian_lin(translation, orientation, speed)

        elif gcode.startswith('M4E'):
            # M4E{1}
            if gcode.endswith('40'):
                self.open_gripper()
            else:
                self.close_gripper()

        else:
            #  Send reply back to client
            # socket.send_string("ok")
            pass

    def home_individual(self):
        for motorName in self.motorNames:
            self.actions.put([(motorName, 0, 1.0)])

    def go_to_cartesian_lin(self, translation, orientation=[0, 0, 0], speed=1.0):
        commands = []
        orientation = [0, 0, 1]
        # orientation = np.eye(3)
        ikResults = self.armChain.inverse_kinematics(translation)
        ikResults = self.armChain.inverse_kinematics(translation, orientation, initial_position=ikResults, orientation_mode="Z")
        # ikResults = self.ik.get_ik(translation, orientation)

        for motorName, i in zip(self.motorNames[:6], range(1, len(ikResults))):
            commands.append((motorName, ikResults[i], speed))

        self.actions.put(commands)

    def increment_cartesian_lin(self, translation, orientation=[0, 0, 0], speed=1.0):
        commands = []
        current_pos = self.get_gripper_position()
        target_position = current_pos + translation
        print(current_pos)
        print(target_position)
        print(translation)

        ikResults = self.armChain.inverse_kinematics(target_position, orientation, orientation_mode="Z")

        for motorName, i in zip(self.motorNames[:6], range(1, len(ikResults))):
            commands.append((motorName, ikResults[i], speed))

        self.actions.put(commands)

    def open_gripper(self):
        commands = [
            ('finger_joint', 0, 1.0),
            ('right_outer_knuckle_joint', 0, 1.0)
        ]

        self.actions.put(commands)

    def close_gripper(self):
        commands = [
            ('finger_joint', 0.4, 1.0),
            ('right_outer_knuckle_joint', 0.4, 1.0)
        ]

        self.actions.put(commands)