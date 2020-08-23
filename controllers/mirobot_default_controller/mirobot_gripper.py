"""MirobotGripper_controller controller."""
# To implement in your main code, have this file in the same directory as your main controller
from math import ceil


class MirobotGripper():
    def __init__(self, robot):
        timestep = int(robot.getBasicTimeStep())
        self.motor_wrist = robot.getMotor('wrist')
        self.wrist_sensor = robot.getPositionSensor('wrist_sensor')
        self.wrist_sensor.enable(timestep)
        self.gripper_sensor = robot.getPositionSensor('gripper_sensor')
        self.gripper_sensor.enable(timestep)
        self.gripper_motors = []
        for name in ['left_outer', 'right_outer', 'right_1', 'right_2', 'left_1', 'left_2']:
            motor = robot.getMotor(name)
            motor.setVelocity(1.0) # use a constant speed for all motors
            self.gripper_motors.append(motor)
        self.gripper_motors[2].enableTorqueFeedback(timestep)
        self.gripper_motors[4].enableTorqueFeedback(timestep) 
        
        self.target_pos = 0 # Closed by default

    def wristMove(self, wrist_pos=0):
        self.motor_wrist.setPosition(wrist_pos)
    
    def gripperPosition(self, gripper_pos=0):
        for motor in self.gripper_motors:
            motor.setPosition(gripper_pos)

    def open(self):
        # for motor in self.gripper_motors:
            # motor.setPosition(1)
        self.target_pos = 1

    def close(self):
        # for motor in self.gripper_motors:
            # motor.setPosition(0)
        self.target_pos = 0
            
    def get_torques(self):
        tq_r = self.gripper_motors[2].getTorqueFeedback()
        tq_l = self.gripper_motors[4].getTorqueFeedback()
        return -(tq_r + tq_l) / 2  
        
    def getSensors(self):
        wrist_pos = self.wrist_sensor.getValue() 
        gripper_pos = self.gripper_sensor.getValue() 
        return [wrist_pos, gripper_pos]
    
    # This function moves the gripper to the "pos_end" position, taking the maxTorque into account
    def gripperMove(self, pos_end, maxTorque, interval_size_rad=0.05):
        pos = pos_start = self.gripper_sensor.getValue() 
        if pos_start < pos_end: # if we are opening the gripper, we ignore the torque, as it can get stuck otherwise
            self.gripperPosition(pos_end)
            return        
        intervals = ceil(abs((pos_end - pos_start) / interval_size_rad))       
        increment = (pos_end - pos_start) / intervals        
        for n in range(intervals):
            if self.get_torques() < maxTorque:
                pos = pos_start +  n * increment            
            for motor in self.gripper_motors:
                motor.setPosition(pos)
            supervisor.step(timestep)
     
    # This function sets the gripper motor positions 1 increment closer to the target location. You will
    # have to call this functions several times for it to reach the new location        
    def gripperMoveIncrement(self, pos_end, maxTorque, increment_rad=0.05):
        pos = pos_start = self.gripper_sensor.getValue() 
        if pos_start < pos_end: # if we are opening the gripper, we ignore the torque, as it can get stuck otherwise
            self.gripperPosition(pos_end)
            return     
        # following 2 lines take your increment_rad and change it slightly, so that we reach the goal exactly   
        intervals = ceil(abs((pos_end - pos_start) / increment_rad)) 
        print(intervals)
        if intervals > 0:      
            increment = (pos_end - pos_start) / intervals
            if self.get_torques() < maxTorque:
                pos = pos_start + increment            
            for motor in self.gripper_motors:
                motor.setPosition(pos)
            
    def run(self):
        print(self.target_pos)
        self.gripperMoveIncrement(self.target_pos, 2)

if __name__ == '__main__':
    from controller import Supervisor
    from math import sin
    # create the Robot instance.
    supervisor = Supervisor()
    # get the time step of the current world.
    timestep = int(supervisor.getBasicTimeStep())
    gripper = MirobotGripper(supervisor)
    while supervisor.step(timestep) != -1:
        t = supervisor.getTime()
        pos = 1.05 * (sin(t)+1)/2 #value looping between 0 and 1
        # MirobotGripper.gripperMoveIncrement(pos, 2)
        if pos > 0.5:
            gripper.open()
        else:
            gripper.close()
            
        gripper.run()
            
        

