from mirobot import Mirobot

with Mirobot(portname='COM3', debug=True) as m:
    # m.home_individual()

    # m.go_to_zero()

    result = m.go_to_cartesian_ptp(10,20,30)

    print(result)