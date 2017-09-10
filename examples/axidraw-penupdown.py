import logging

from axidrawcontrol import AxidrawControl, AxidrawError

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    try:
        with AxidrawControl() as axidraw:
            axidraw.setup_servo(0, 1, 2,2)
            axidraw.pen_up()
            axidraw.pen_down()
            axidraw.pen_up()

            axidraw.setup_servo(.5, 1, 2,2)
            axidraw.pen_down()
            axidraw.pen_up()

            axidraw.setup_servo(.5, 1, 1,1)
            axidraw.pen_down()
            axidraw.pen_up()


    except AxidrawError as e:
        print('error:', e)
