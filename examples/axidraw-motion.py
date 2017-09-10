import logging

from axidrawcontrol import AxidrawControl, AxidrawError

def read_trajectory(path):
    for line in open(path):
        yield [float(x) for x in line.strip().split(' ')]


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)

    logging.getLogger('axidrawcontrol.ebb').setLevel(logging.WARN)
    logging.getLogger('axidrawcontrol.motionplanning').setLevel(logging.WARN)
    try:
        xys = list(read_trajectory('trajectory1.txt'))

        with AxidrawControl() as axidraw:
            axidraw.setup_servo(0, 1, 2,2)
            axidraw.enable_motors(highres=True)

            axidraw.pen_down()
            axidraw.move_to(xys, (1,1,.75))
            axidraw.pen_up()

            axidraw.enable_motors(highres=False)
            axidraw.pen_down()
            axidraw.move_to(xys, (1,1,.75))
            axidraw.pen_up()

            axidraw.park()
            axidraw.disable_motors()


    except AxidrawError as e:
        print('error:', e)
