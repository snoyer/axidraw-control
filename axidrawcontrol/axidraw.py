from collections import namedtuple
import math
import itertools
import logging

import numpy as np

from .ebb import EBB, EBBSerialError
from . import motionplanning


logger = logging.getLogger(__name__)



'''Hardware-related constants'''
SERVO_MIN =  7500    # lowest pen position in servo units
SERVO_MAX = 28000    # highest pen position in servo units
MIN_MOTOR_RATE =     4    # minimum motor rate in Hz (steps/s)
MAX_MOTOR_RATE = 25000    # maximum motor rate in Hz (steps/s)
STEPS_PER_MM_AT_1X = 5    # number of motor units (steps) per millimiter without microstepping
LM_FREQUENCY = 25000    # low-level move motor frequency in Hz

LM_RATE_FACTOR = (2**31)/LM_FREQUENCY    # low-level rate factor
SERVO_SPEED_UNIT = (SERVO_MAX - SERVO_MIN) * 24 / 1000

MIN_ACCELERATION =   5    # mm/s/s
MAX_ACCELERATIONS = {4: 1000, 8: 4000, 16: 8000}    # microstepping -> mm/s/s
SAFE_ACCELERATIONS = {4: 1000, 8: 1200, 16: 1200}    # microstepping -> mm/s/s

SQRT2 = math.sqrt(2)



class EbbMixin(object):
    def __init__(self, port=None, EbbClass=EBB, *args, **kwargs):
        self.ebb = EbbClass(port=port)
        super().__init__(*args, **kwargs)


    def ebb_run(self, *commands):
        return self.ebb.run(*commands)


    def __enter__(self):
        #TODO file lock maybe?
        return self

    def __exit__(self, type, value, traceback):
        self.ebb.close()



class MotorsMixin(object):
    def __init__(self, microstepping=16, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.microstepping = microstepping

    @property
    def microstepping(self):
        return self._microstepping
    
    @microstepping.setter
    def microstepping(self, microstepping):
        possible_values = [16,8,4]  # 2x or full step doesn't seem to run smooth at all
        if not microstepping in possible_values:
            raise ValueError('microstepping must be in %s' % possible_values)
        self._microstepping = microstepping

        # update microstepping on EBB
        self.enable_motors()
        self.disable_motors()


    @property
    def steps_per_mm(self):
        return STEPS_PER_MM_AT_1X * self.microstepping


    def project(self, xy):
        x,y = xy.real, xy.imag
        ij = complex(x+y, x-y)
        return ij * self.steps_per_mm

    def unproject(self, ij):
        i,j = ij.real, ij.imag
        xy = complex(i+j, i-j) / 2
        return xy / self.steps_per_mm


    @property
    def min_speed(self):
        return MIN_MOTOR_RATE / self.steps_per_mm / SQRT2    # mm/s

    @property
    def max_speed(self):
        return MAX_MOTOR_RATE / self.steps_per_mm / SQRT2    # mm/s


    def motionplanning_constraints(self, speed_settings):
        speed_t, accel_t, cornering_t = speed_settings

        v_max = lerp_clamp(speed_t, 0,1, self.min_speed, self.max_speed)
        accel = clamp(lerp(accel_t, 0,1, MIN_ACCELERATION, SAFE_ACCELERATIONS[self.microstepping]),
                      MIN_ACCELERATION, MAX_ACCELERATIONS[self.microstepping])
        cornering = lerp_clamp(cornering_t, 0,1, 0, 1) #TODO figure out correct bounds
        return motionplanning.Constraints(
            v_max = v_max,
            accel = accel, decel = accel,
            cornering_tolerance = cornering
        )


    def enable_motors(self, enable=True):
        if enable:
            step_mode = 5-int(math.log2(self.microstepping))
        else:
            step_mode = 0
        self.ebb_run(('EM', step_mode, step_mode))

    def disable_motors(self, disable=True):
        return self.enable_motors(not disable)

    def wait_until_stopped(self):
        while self.ebb.run('QM') != b'QM,0,0,0,0\n\r':
            pass


    def LM_move(self, di,dj, v0, v1):
        if di or dj:
            l = abs(complex(di,dj)) # displacement norm
            li = abs(di/l) # normalized i component
            lj = abs(dj/l) # normalized j component
            self.ebb_run(('LM', *self.LM_axis_params(di, v0*li, v1*li),
                                *self.LM_axis_params(dj, v0*lj, v1*lj)))

    @staticmethod
    def LM_axis_params(steps, vs0, vs1):
        if steps == 0:
            return 0,0,0

        rate0 = round(LM_RATE_FACTOR * vs0)
        rate1 = round(LM_RATE_FACTOR * vs1)

        duration = 2 * abs(steps) / (vs0+vs1)
        delta = round((rate1-rate0) / (duration*LM_FREQUENCY))
        
        return rate0, steps, delta



class SimpleServoMixin(object):
    def __init__(self, pen_up_pos=1, pen_down_pos=0, pen_up_speed = 2, pen_down_speed = 2, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.pen_up_pos = pen_up_pos
        self.pen_down_pos = pen_down_pos
        self.pen_up_speed = pen_up_speed
        self.pen_down_speed = pen_down_speed
        self.pen_is_up = None


    @property
    def pen_up_pos(self):
        return self._pen_up_pos
    
    @pen_up_pos.setter
    def pen_up_pos(self, pen_up_pos):
        self.ebb_run(('SC', 4, lerp(pen_up_pos, 0,1, SERVO_MIN,SERVO_MAX)))
        self._pen_up_pos = pen_up_pos


    @property
    def pen_down_pos(self):
        return self._pen_down_pos

    @pen_down_pos.setter
    def pen_down_pos(self, pen_down_pos):
        self.ebb_run(('SC', 5, lerp(pen_down_pos, 0,1, SERVO_MIN,SERVO_MAX)))
        self._pen_down_pos = pen_down_pos


    @property
    def pen_up_speed(self):
        return self._pen_up_speed
    
    @pen_up_speed.setter
    def pen_up_speed(self, pen_up_speed):
        self.ebb_run(('SC', 11, pen_up_speed * SERVO_SPEED_UNIT))
        self._pen_up_speed = pen_up_speed


    @property
    def pen_down_speed(self):
        return self._pen_down_speed
    
    @pen_down_speed.setter
    def pen_down_speed(self, pen_down_speed):
        self.ebb_run(('SC', 12, pen_down_speed * SERVO_SPEED_UNIT))
        self._pen_down_speed = pen_down_speed


    @property
    def safe_pen_down_delay(self):
        return abs(self.pen_up_pos - self.pen_down_pos) / self.pen_down_speed

    @property
    def safe_pen_up_delay(self):
        return abs(self.pen_up_pos - self.pen_down_pos) / self.pen_up_speed


    def raise_pen(self, delay=None):
        if self.pen_is_up:
            return
        self.pen_is_up = True

        if delay == None:
            delay = self.safe_pen_up_delay
        self.ebb_run(('SP', 1, delay*1000))


    def lower_pen(self, delay=None):
        if not self.pen_is_up:
            return
        self.pen_is_up = False

        if delay == None:
            delay = self.safe_pen_down_delay
        self.ebb_run(('SP', 0, delay*1000))
        self.pen_is_up = False



class VariableServoMixin(object):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # self.pen_up_pos = None
        # self.pen_down_pos = None
        self.pen_current_pos = 1


    def move_pen(self, height, duration=None, delay=None, speed=2):
        # do some trig to get linear movement from axial servo
        new_pos = math.asin(clamp(height, 0, 1)*2-1)/math.pi+.5

        delta = abs(self.pen_current_pos - new_pos)
        if delta < 0.01:
            return
        
        if duration:
            speed = delta / duration
        elif speed:
            duration = delta / speed

        else:
            raise ValueError('must provide duration or speed')

        if delay == None:
            delay = duration

        sp,sr,p = (4,11,1) if new_pos < self.pen_current_pos else (5,12,0)
        self.ebb_run(
            ('SC', sp, lerp(new_pos, 0,1, SERVO_MIN,SERVO_MAX)),
            ('SC', sr, speed * SERVO_SPEED_UNIT),
            ('SP', p, delay * 1000)
        )
        self.pen_current_pos = new_pos



class PenMixin(MotorsMixin, SimpleServoMixin):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.current_position = 0j


    def draw(self, xys, speed_settings):
        self.move_to(xys[:1], speed_settings)
        self.lower_pen()
        self.move_to(xys[1:], speed_settings)
        self.raise_pen()


    def move_to(self, xys, speed_settings):
        l = (len(xys)+1) if hasattr(xys, '__len__') else -1

        xys = np.fromiter(map(make_complex, itertools.chain([self.current_position],xys)), np.complex128, l)
        vectors = np.ediff1d(xys)

        self.move_by(vectors, speed_settings)


    def move_by(self, vectors, speed_settings):

        norms = np.abs(vectors)
        normed_vectors = vectors/norms

        constraints = self.motionplanning_constraints(speed_settings)
        profile = list(motionplanning.concat_blocks(motionplanning.velocity_profile(constraints, normed_vectors, norms)))
        if profile:
            ts,vs,ds = zip(*profile)
            positions = np.interp(ds, np.hstack((0, np.cumsum(norms))),
                                      np.hstack((0, np.cumsum(vectors))))
            
            speed_factor = self.steps_per_mm * SQRT2 # to convert speed in steps/s
            ei,ej = 0,0
            for (v0,p0),(v1,p1) in pairwise(zip(vs,positions)):
                ij = self.project(p1)-self.project(p0) # displacement in steps
                ei,di = roundf(ij.real+ei) # round i component
                ej,dj = roundf(ij.imag+ej) # round j component

                self.LM_move(di, dj, v0*speed_factor, v1*speed_factor)
                self.current_position += self.unproject(complex(di,dj))



class BrushpenMixin(MotorsMixin, VariableServoMixin):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.current_position = 0j


    def draw(self, xyzs, speed_settings):
        xys =[]
        zs = []
        for xyz in xyzs:
            xys.append(xyz[:2])
            zs.append(xyz[2] if len(xyz)>2 else None)

        pen_pos = self.pen_current_pos

        #TODO improve initial move
        # should be able to move short of the first point and have a smooth transifition from initial pen position to first point
        self.move_to(xys[:1], speed_settings)
        if zs[0]:
            self.move_pen(zs[0], .2)

        self.move_to(xys[1:], speed_settings, zs[1:])
        self.move_pen(pen_pos, .2)


    def move_to(self, xys, speed_settings, zs=None):
        l = (len(xys)+1) if hasattr(xys, '__len__') else -1

        xys = np.fromiter(map(make_complex, itertools.chain([self.current_position],xys)), np.complex128, l)
        vectors = np.ediff1d(xys)

        self.move_by(vectors, speed_settings, zs)


    def move_by(self, vectors, speed_settings, zs=None):

        norms = np.abs(vectors)
        normed_vectors = vectors/norms

        constraints = self.motionplanning_constraints(speed_settings)
        blocks = list(motionplanning.velocity_profile(constraints, normed_vectors, norms))

        def zmoves():
            if zs and any(z!=None for z in zs):
                block_ts = [block[0][0] if block else 0 for block in blocks]
                for (i0,z0),(i1,z1) in pairwise((i,z) for i,z in enumerate(zs) if z!=None or i==0):
                    yield z1, block_ts[i1]-block_ts[i0]
                    for k in range(i1-i0-1):
                        yield None
                yield None
            else:
                yield from itertools.repeat(None, len(vectors))

        ei,ej = 0,0 # step rounding errors
        speed_factor = self.steps_per_mm * SQRT2 # to convert speed in steps/s
            
        norms_cumsum = np.hstack((0, np.cumsum(norms)))
        vectors_cumsum = np.hstack((0, np.cumsum(vectors)))

        for block,zmove in zip(blocks, zmoves()):
            if zmove:
                z,dt = zmove
                self.move_pen(z, dt, 0)

            if block:
                ts,vs,ds = zip(*block)
                xys = np.interp(ds, norms_cumsum, vectors_cumsum)

                for (t0,v0,xy0),(t1,v1,xy1) in pairwise(zip(ts,vs,xys)):
                    ij = self.project(xy1)-self.project(xy0) # displacement in steps
                    ei,di = roundf(ij.real+ei) # round i component
                    ej,dj = roundf(ij.imag+ej) # round j component

                    self.LM_move(di, dj, v0*speed_factor, v1*speed_factor)
                    self.current_position += self.unproject(complex(di,dj))




class Axidraw(EbbMixin, PenMixin):
    pass




class Axidraw2(EbbMixin, BrushpenMixin):
    pass




def make_complex(xy):
    return xy if isinstance(xy, complex) else complex(*xy)


def roundf(x):
    xi = round(x)
    return x - xi, xi


def clamp(x, low, high):
    return low  if x < low  \
      else high if x > high \
      else x


def lerp(t, t0, t1, v0, v1):
    tv = (t - t0) / (t1 - t0)
    return v0 + (v1 - v0) * tv


def lerp_clamp(t, t0, t1, v0, v1):
    return clamp(lerp(t, t0,t1, v0,v1), v0,v1)



def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)