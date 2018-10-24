import os
import logging

from axidrawcontrol import Axidraw, EBBSerialError

def read_trajectory(path):
    for line in open(path):
        yield [float(x) for x in line.strip().split(' ')]


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)

    logging.getLogger('axidrawcontrol.ebb').setLevel(logging.WARN)
    logging.getLogger('axidrawcontrol.motionplanning').setLevel(logging.WARN)
    try:
        xys = list(read_trajectory(os.path.join(os.path.dirname(__file__), 'trajectory1.txt')))

        speed_settings = .5,.5,.1
        with Axidraw() as axidraw:
            axidraw.pen_up_pos = 1
            axidraw.pen_up_speed = 2
            axidraw.raise_pen()

            axidraw.microstepping = 16
            axidraw.move_to(xys, speed_settings)
            axidraw.wait_until_stopped()

            axidraw.microstepping = 8
            axidraw.move_to(xys, speed_settings)
            axidraw.wait_until_stopped()

            axidraw.microstepping = 4
            axidraw.move_to(xys, speed_settings)
            axidraw.wait_until_stopped()

            axidraw.microstepping = 16
            axidraw.move_to([(0,0)], speed_settings)
            axidraw.wait_until_stopped()

            axidraw.disable_motors()

            print(axidraw.current_position)

    except EBBSerialError as e:
        print('error:', e)
