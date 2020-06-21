import os, sys
from collections import namedtuple
import copy
import logging


from .ebb import parse_commands, stepmode_to_microstepping
from .ebb import SERVO_MAX, SERVO_MIN, LM_FREQUENCY
from .axidraw import STEPS_PER_MM_AT_1X, angle_to_pen_height
from .util.misc import lerp





class EbbState(object):

    def __init__(self):
        self.version = b'EBBv13_and_above EB Firmware Version 0.0.0'
        self.nickname = b'simulator'

        self.stepmode = 1
        self.axis1enabled = False
        self.axis2enabled = False
        self.stepcounter1 = 0
        self.stepcounter2 = 0
        
        self.servo_min = SERVO_MIN
        self.servo_max = SERVO_MAX
        self.servo_rate_up = 0
        self.servo_rate_down = 0

        self.pen_down = False
        self.layer = 0

        self.ra0_voltage = 3
        self.vplus_voltage = 3

        self.OK = b'OK\r\n'

        self.timer_time = 0
        pos = SERVO_MIN if self.pen_down else SERVO_MAX
        self._last_servo_move = (-1,pos, 0,pos)

    @property
    def servo_pos(self):
        t = self.timer_time
        t0,v0,t1,v1 = self._last_servo_move
        if t >= t1:
            return v1
        if t <= t0:
            return v0
        return lerp(t, t0,t1, v0,v1)

    @property
    def expects_OK(self):
        return self.OK != b''
    


    def ebb_V(self):
        return self.version + b'\r\n'

    def ebb_QT(self):
        return self.nickname + b'\r\n' + self.OK


    def ebb_CU(self, value1, value2):
        if int(value1) == 1:
            self.OK = b'OK\r\n' if int(value2) == 1 else b''
        return self.OK

    def ebb_EM(self, enableAxis1, enableAxis2=None):
        enableAxis1 = int(enableAxis1)
        enableAxis2 = int(enableAxis2) if enableAxis2!=None else enableAxis1
        self.axis1enabled = enableAxis1 > 0
        self.axis2enabled = enableAxis2 > 0
        if enableAxis1 > 0:
            self.stepmode = enableAxis1
        return self.OK


    def ebb_QM(self, *args):
        return b'QM,0,0,0,0\n\r'


    def ebb_QB(self, *args):
        return b'0\r\n' + self.OK


    def ebb_SC(self, value1, value2):
        value1 = int(value1)
        value2 = int(value2)
        if value1 == 4:
            self.servo_min = value2
        elif value1 == 5:
            self.servo_max = value2
        elif value1 == 11:
            self.servo_rate_up = value2
        elif value1 == 12:
            self.servo_rate_down = value2
        else:
            logging.warn('simulation not implemented for SC %d', value1)

        return self.OK


    def ebb_QC(self):
        return b'%04d,%04d\r\n' % (
            int(lerp_clamp(self.ra0_voltage  , 0,3.3, 0,1023)),
            int(lerp_clamp(self.vplus_voltage, 0,3.3, 0,1023))
        ) + self.OK


    def ebb_SL(self, value):
        self.layer = int(value)
        return self.OK

    def ebb_QL(self):
        return b'%d\r\n' % self.layer + self.OK


    def ebb_SP(self, value, delay=0, portBpin=None):
        down = int(value) == 0

        pos = self.servo_pos
        if down:
            new_pos = self.servo_max
            rate = self.servo_rate_down
        else:
            new_pos = self.servo_min
            rate = self.servo_rate_up

        if rate:
            duration = abs(new_pos-pos)/rate * 24/1000 #TODO double check
        else:
            duration = 0

        t = self.timer_time
        self._last_servo_move = t, pos, t+duration, new_pos
        self.timer_time = t + delay/1000

        self.pen_down = down
        return self.OK

    def ebb_TP(self, value, delay=0):
        return self.SP(1 if self.pen_down else 0, delay)

    def ebb_QP(self):
        return b'%d\r\n' % (0 if self.pen_down else 1) + self.OK


    def ebb_SM(self, duration, axisSteps1, axisSteps2=0):
        self.axis1enabled = True
        self.axis2enabled = True

        self.stepcounter1 += int(axisSteps1)
        self.stepcounter2 += int(axisSteps2)

        self.timer_time += duration/1000

        return self.OK



    def ebb_XM(self, duration, axisSteps1, axisSteps2=0):
        self.axis1enabled = True
        self.axis2enabled = True

        self.stepcounter1 += int(axisSteps1+axisSteps2)
        self.stepcounter2 += int(axisSteps1-axisSteps2)

        self.timer_time += duration/1000

        return self.OK


    def ebb_LM(self, rateTerm1, axisSteps1, deltaR1, rateTerm2, axisSteps2, deltaR2):
        self.axis1enabled = True
        self.axis2enabled = True

        self.stepcounter1 += int(axisSteps1)
        self.stepcounter2 += int(axisSteps2)
        
        duration = lm_sim_time(rateTerm1, axisSteps1, deltaR1, rateTerm2, axisSteps2, deltaR2)
        self.timer_time += duration
        
        return self.OK




    def run(self, x):
        response = b''
        for command in parse_commands(x):
            f = getattr(self, 'ebb_'+command[0].upper(), None)
            if f:
                r = f(*command[1:])
                if r != None:
                    response += r
            else:
                logging.warn('simulation not implemented for %s', repr(command[0]))
                response += b'!err: not implemented\r\n'

        return response


    def simulate(self, commands, state_copy_func=None, include_initial=False):
        if not state_copy_func:
            state_copy_func = copy.copy
        
        if include_initial:
            yield Simulated(b'', b'', state_copy_func(self))
        for command in commands:
            response = self.run(command)
            yield Simulated(command, response, state_copy_func(self))


Simulated = namedtuple('Simulated', 'command response state')



class AxidrawState(EbbState):
    def __init__(self):
        super().__init__()
        self.pen_position = 0j

    @property
    def microstepping(self):
        return stepmode_to_microstepping(self.stepmode)
    
    
    @property
    def pen_height(self):
        return angle_to_pen_height((self.servo_pos - SERVO_MIN) / (SERVO_MAX-SERVO_MIN))
    
    @property
    def pen_up_height(self):
        return angle_to_pen_height((self.servo_min - SERVO_MIN) / (SERVO_MAX-SERVO_MIN))

    @property
    def pen_down_height(self):
        return angle_to_pen_height((self.ebb.servo_max - SERVO_MIN) / (SERVO_MAX-SERVO_MIN))


    def ebb_SM(self, duration, axisSteps1, axisSteps2=0):
        response = super().ebb_SM(duration, axisSteps1, axisSteps2)
        self.pen_position += self.unproject(complex(axisSteps1, axisSteps2))
        return response

    def ebb_XM(self, duration, axisSteps1, axisSteps2=0):
        response = super().ebb_XM(duration, axisSteps1, axisSteps2)
        self.pen_position += self.unproject(complex(axisSteps1 + axisSteps2,
                                                        axisSteps1 - axisSteps2))
        return response

    def ebb_LM(self, *args):
        response = super().ebb_LM(*args)
        rateTerm1, axisSteps1, deltaR1, rateTerm2, axisSteps2, deltaR2 = map(int, args)
        duration = lm_sim_time(rateTerm1, axisSteps1, deltaR1, rateTerm2, axisSteps2, deltaR2)
        self.pen_position += self.unproject(complex(axisSteps1,axisSteps2))
        return response
    

    @property
    def steps_per_mm(self):
        return STEPS_PER_MM_AT_1X * stepmode_to_microstepping(self.stepmode)

    def project(self, xy):
        x,y = xy.real, xy.imag
        ij = complex(x+y, x-y)
        return ij * self.steps_per_mm

    def unproject(self, ij):
        i,j = ij.real, ij.imag
        xy = complex(i+j, i-j) / 2
        return xy / self.steps_per_mm



def lm_sim_time(rate1, steps1, delta1, rate2, steps2, delta2):
    steps1 = abs(steps1)
    steps2 = abs(steps2)
    acc1 = s1 = 0
    acc2 = s2 = 0

    n = 0

    axis1 = axis2 = True
    while axis1 or axis2:
        if axis1:
            acc1 = (acc1 + rate1) % 0x100000000
            rate1 += delta1
            if acc1 >= 0x80000000:
                acc1 -= 0x80000000
                s1 += 1
            axis1 = s1 < steps1

        if axis2:
            acc2 = (acc2 + rate2) % 0x100000000
            rate2 += delta2
            if acc2 >= 0x80000000:
                acc2 -= 0x80000000
                s2 += 1
            axis2 = s2 < steps2
        n +=1

    return n/LM_FREQUENCY

