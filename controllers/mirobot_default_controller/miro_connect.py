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
    m.go_to_cartesian_lin(x=260, y=0, z=100, a=0.000, b=0.000, c=0.000)

    # m.go_to_cartesian_lin(x=250, y=0, z=150, a=0.000, b=0.000, c=0.000)
    # time.sleep(1)

    m.set_valve(65) # close to grab the blue box

    # m.increment_cartesian_lin(z=150) # go up
    # m.go_to_cartesian_lin(x=270, y=200, z=30, a=0.000, b=0.000, c=0.000)

    # m.set_valve(40)

    m.go_to_cartesian_lin(x=200, y=0, z=150, a=0.000, b=0.000, c=0.000)
    m.go_to_cartesian_lin(x=200, y=0, z=120, a=0.000, b=0.000, c=0.000)
    m.set_valve(40) # open
    # time.sleep(1)

    

    

    print('finished')