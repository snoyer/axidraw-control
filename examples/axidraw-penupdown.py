import logging

from axidrawcontrol import Axidraw, EBBSerialError

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    try:
        with Axidraw() as axidraw:
            axidraw.pen_up_pos = 1
            axidraw.pen_up_speed = 2
            axidraw.pen_down_pos = 0
            axidraw.pen_down_speed = 2
            axidraw.raise_pen()
            axidraw.lower_pen()
            axidraw.raise_pen()

            axidraw.pen_up_pos = 1
            axidraw.pen_up_speed = 2
            axidraw.pen_down_pos = .5
            axidraw.pen_down_speed = 2
            axidraw.lower_pen()
            axidraw.raise_pen()

            axidraw.pen_up_pos = 1
            axidraw.pen_up_speed = 1
            axidraw.pen_down_pos = .5
            axidraw.pen_down_speed = 1
            axidraw.lower_pen()
            axidraw.raise_pen()


    except EBBSerialError as e:
        print('error:', e)
