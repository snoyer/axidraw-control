import math
from collections import namedtuple
import itertools
import logging

import numpy as np

from . import motionplanning
from .ebb import EBB, EBBSerialError

logger = logging.getLogger(__name__)


SpeedSettings = namedtuple('SpeedSettings', 'speed accel cornering')


'''Hardware-related constants'''
SERVO_MIN = 7500   # lowest pen position in servo units
SERVO_MAX = 28000  # highest pen position in servo units
STEPS_PER_MM_AT_16X = 80  # number of motor units (steps) per milimiter at max microstepping
MIN_MOTOR_SPEED = 4      # minimum motor speed in steps/s
MAX_MOTOR_SPEED = 25000  # maximum motor speed in steps/s
MAX_ACCELERATION = 160000 # acceleration rate in steps/s/s
MIN_ACCELERATION = 10000



class AxidrawError(EBBSerialError):
    def __init__(self, message):
        # replace `EBB` with `Axidraw` in error text for more user-friendly message
        EBBSerialError.__init__(self, message.replace('EBB', 'Axidraw'))


class AxidrawControl(object):

    def __init__(self, port=None, timeout=1.0):
        try:
            self._ebb = EBB(port, timeout)
        except EBBSerialError as e:
            raise AxidrawError(str(e))

        self._pen_up_pos = None
        self._pen_down_pos = None
        self._pen_up_speed = None
        self._pen_down_speed = None

        self._microstepping_scale = None

        self._steps_counter = 0+0j

    def query_version(self):
        """Query the EBB version.

        :return: EBB version string
        """
        return EBB.decode(self._ebb.run('V')).strip()

    def query_button(self):
        return EBB.decode(self._ebb.run('QB')).strip().startswith('1')


    def setup_servo(self, down_pos, up_pos, down_speed, up_speed):
        """Set the positions and speed for pen movements.

        :param down_pos: pen down position as interpolant of servo range
        :param up_pos:   pen up position as interpolant of servo range
        :param down_speed: pen down speed in range/second
        :param up speed:   pen up speed in range/second
        """

        up_pos = clamp(up_pos, 0, 1)
        down_pos = clamp(down_pos, 0, 1)

        # the EBB takes speeds in steps (1/12 MHz) per 24 ms.
        speed_unit = (SERVO_MAX - SERVO_MIN) * 24 / 1000.

        self._ebb.run(
            ('SC',4, lerp(  up_pos, 0,1, SERVO_MIN,SERVO_MAX)),
            ('SC',5, lerp(down_pos, 0,1, SERVO_MIN,SERVO_MAX)),
            ('SC',11, speed_unit *   up_speed),
            ('SC',12, speed_unit * down_speed),
        )

        self._pen_up_pos = up_pos
        self._pen_down_pos = down_pos
        self._pen_up_speed = up_speed
        self._pen_down_speed = down_speed

    def safe_pen_up_delay(self):
        """Compute duration for pen up movement based on servo configuration.

        :return: duration in second
        """
        if not self._pen_up_speed:
            raise AxidrawError('servo not configured yet')
        return (self._pen_up_pos - self._pen_down_pos) / self._pen_up_speed

    def safe_pen_down_delay(self):
        """Compute duration for pen down movement based on servo configuration.

        :return: duration in second
        """
        if not self._pen_down_speed:
            raise AxidrawError('servo not configured yet')
        return (self._pen_up_pos - self._pen_down_pos) / self._pen_down_speed


    def is_pen_up(self):
        """Query the EBB to see if the pen is up."""
        return self._ebb.run('QP').startswith(b'1')

    def toggle_pen(self):
        """Send a toggle pen command to the EBB."""
        self._ebb.run('TP')

    def pen_up(self, delay=None):
        """Send a "pen up" command to the EBB.
        if no `delay` value is passed, a default will be computed to allow the full move to be performed

        :param delay: delay in seconds
        """
        if delay == None:
            delay = self.safe_pen_up_delay()
        self._ebb.run(('SP', 1, delay*1000))

    def pen_down(self, delay=None):
        """Send a "pen down" command to the EBB.
        if no `delay` value is passed, a default will be computed to allow the full move to be performed

        :param delay: delay in seconds
        """
        if delay == None:
            delay = self.safe_pen_down_delay()
        self._ebb.run(('SP', 0, delay*1000))


    def disable_motors(self):
        """Send a ""disable motors' command to the EBB."""
        self._ebb.run(('EM', 0, 0))

    def enable_motors(self, highres=True):
        """Send a ""enable motors' command to the EBB.
        The micro-stepping will be set to 16x if `highres` is `True` (default) or 8x otherwise

        :param highres: wether to use 16x or 8x micro-stepping
        """
        res = 1 if highres else 2
        self._ebb.run(('EM', res, res))

        old_microstepping_scale = self._microstepping_scale

        self._microstepping_scale = 2**(res-1)

        # update step counters in case we change resolution multiple times
        if old_microstepping_scale and old_microstepping_scale != self._microstepping_scale:
            factor = old_microstepping_scale / self._microstepping_scale
            self._steps_counter *= factor


    def query_motors(self):
        """Query the EBB for motors status.

        :return: a list of 4 booleans
        """
        response = EBB.decode(self._ebb.run('QM')).strip()
        return [token=='1' for token in response.split(',')[1:]]

    def query_steps(self):
        """Query the EBB for step position.

        :return: a (motor1 steps, motor2 steps) tuple
        """
        response = EBB.decode(self._ebb.run('QS')).strip()
        try:
            split = response.split('\n', 1)[0].split(',')
            return int(split[0]), int(split[1])
        except ValueError:
            raise AxidrawError('could not query step position')


    def move_to(self, xys, speed_settings):
        """Perform movements in user space.

        :param xys: squence of `(x,y)` coordinates *in mm* defining the trajectory to follow.
        :param speed_settings: `SpeedSettings` namedtuple
        """

        xys_steps = np.fromiter(map(self.project_to_motor_steps, xys), np.complex128, len(xys))
        xys_steps = np.around(xys_steps)

        vectors = np.ediff1d(xys_steps)
        vectors = np.hstack((xys_steps[0]-self._steps_counter, vectors))

        self.move_by_steps(vectors, self.motion_constraints(*speed_settings))


    def park(self):
        """Lift pen, move back home, disable motors."""

        self.pen_up(self.safe_pen_up_delay())

        self.move_by_steps([-self._steps_counter], self.motion_constraints(1., 1., 0))

        logger.debug('waiting for end of motion')
        while any(self.query_motors()):
            pass

        self.disable_motors()
    

    def move_by_steps(self, vectors, motion_constraints):
        """Perform pen movements in hardware space.

        :param vectors: relative displacement vectors in motor steps as numpy array of complex numbers
        :param motion_constraints: motion planning constraints with:
            - velocity in steps/s
            - acceleration/deceleration rates in steps/s/s
            - cornering tolerance in deep-magic units?
        """

        norms = np.abs(vectors)
        normed_vectors = vectors/norms

        profile = list(motionplanning.velocity_profile(normed_vectors, norms, motion_constraints))

        if profile:
            interpolated_profile = motionplanning.interpolate_velocity_profile(
                profile, timestep=0.025
            )
            ts,vs,ds = zip(*interpolated_profile)

            positions = np.interp(ds, np.hstack((0, np.cumsum(norms))),
                                      np.hstack((0, np.cumsum(vectors)))
            )
            positions = np.around(positions)

            for dt,dp in zip(np.ediff1d(ts), np.ediff1d(positions)):
                if dp.real or dp.imag:
                    self._ebb.run(('SM', dt*1000, dp.real, dp.imag))
                    self._steps_counter += dp


    def motion_constraints(self, speed=.5, accel=.5, cornering=.5):
        """Compute hadrware motion planning constraints from user speed settings

        Parameters are expected as interpolant values between `0.` for slow and `1.` for fast.

        :param speed: speed interpolant value
        :param accel: acceleration interpolant value
        :param cornering: cornering tolerance interpolant value
        """

        return motionplanning.Constraints(
            v_max = lerp(speed, 0,1, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED),
            accel = lerp(accel, 0,1, MIN_ACCELERATION, MAX_ACCELERATION) / self._microstepping_scale,
            decel = lerp(accel, 0,1, MIN_ACCELERATION, MAX_ACCELERATION) / self._microstepping_scale,
            cornering_tolerance = lerp(cornering, 0,1, 0.1, 1), #TODO figure out correct bounds
        )


    def project_to_motor_steps(self, xy_mm):
        """Project a point from user-space (mm) to hardware space (motor steps)

        :param xy_mm: `(x,y)` coordinates in milimeters, as a tuple or complex number
        :return: `(steps1,steps2)` coordinate as a complex number
        """
        if isinstance(xy_mm, complex):
            x,y = xy_mm.real, xy_mm.imag
        else:
            x,y = tuple(xy_mm)

        if not self._microstepping_scale:
            raise AxidrawError('motors not enabled')
        steps_per_mm = STEPS_PER_MM_AT_16X / self._microstepping_scale

        x_steps = x * steps_per_mm
        y_steps = y * steps_per_mm
        a_steps = x_steps + y_steps
        b_steps = x_steps - y_steps

        return complex(a_steps, b_steps)


    def __str__(self):
        return '<AxidrawControl on %s>' % self._ebb.port()


    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self._ebb.close()




def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)

def clamp(x, low, high):
    return low  if x < low  \
      else high if x > high \
      else x

def lerp(t, t0, t1, v0, v1):
    tv = (t - t0) / (t1 - t0)
    return v0 + (v1 - v0) * tv
