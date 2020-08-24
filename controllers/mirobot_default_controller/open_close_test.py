from mirobot import Mirobot
import time

# first = [0.27, 0, 0.030]
# second = [0.200, 0, 0.150]
# controller.open_gripper()
# controller.go_to_cartesian_lin(first)
# controller.close_gripper()
# controller.go_to_cartesian_lin(second)

with Mirobot(portname='COM3', connection_type='ip', debug=True) as m:
    # m.home_individual()

    # m.go_to_zero()

    m.set_valve(40) # open

    m.set_valve(65)

    

    # m.set_valve(65) # close to grab the blue box
    # time.sleep(1)

    

    

    print('finished')