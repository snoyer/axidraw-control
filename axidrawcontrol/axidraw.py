from collections import namedtuple
import math
from itertools import chain, takewhile
import logging

import numpy as np
import rdp

from .ebb import LM_command, SC_commands, enable_motors_command, disable_motors_command
from . import motionplanning
from .util.svg import path_to_polylines as svg_path_to_polylines
from .util.misc import pairwise, lerp, clamp, lerp_clamp, croundf, uniq


logger = logging.getLogger(__name__)



'''Hardware-related constants'''
MIN_MOTOR_RATE =     4    # minimum motor rate in Hz (steps/s)
MAX_MOTOR_RATE = 25000    # maximum motor rate in Hz (steps/s)
STEPS_PER_MM_AT_1X = 5    # number of motor units (steps) per millimiter without microstepping

MIN_ACCELERATION =   5    # mm/s/s
MAX_ACCELERATIONS = {4: 1000, 8: 2000, 16: 4000}    # microstepping -> mm/s/s
MIN_CORNERING = 0.0001
MAX_CORNERING = 0.01

MIN_PEN_SPEED = .2 #TODO find actual values
MAX_PEN_SPEED = 1.5  #TODO find actual values

MIN_TIMESTEP      = 0.005
PREFERED_TIMESTEP = 0.025

SQRT2 = math.sqrt(2)


_MotionSettings = namedtuple('MotionSettings', '''
velocity_up,
acceleration_up,
deceleration_up,
cornering_up,

velocity_down,
acceleration_down,
deceleration_down,
cornering_down,

height_up,
height_down,
height_pressure,

lowering_speed,
raising_speed,
''')


def MotionSettings(**kwargs):
    def find(*args, default=None):
        for arg in args:
            v = kwargs.get(arg)
            if v != None:
                return v
        return default


    velocity_down = find('velocity_down', 'velocity', default=.5)
    acceleration_down = find('acceleration_down', 'acceleration', default=.5)
    deceleration_down = find('deceleration_down', default=acceleration_down)
    cornering_down = find('cornering_down', 'cornering', default=.5)

    velocity_up = find('velocity_up', default=velocity_down)
    acceleration_up = find('acceleration_up', default=acceleration_down)
    deceleration_up = find('deceleration_up', default=deceleration_down)
    cornering_up = find('cornering_up', default=cornering_down)

    height_up =  find('height_up', 'pen_up', default=1)
    height_down = find('height_down', 'pen_down', default=0)
    height_pressure = find('height_pressure', default=height_down)

    lowering_speed = find('lowering_speed', 'pen_speed', default=1)
    raising_speed = find('raising_speed', default=lowering_speed)

    return _MotionSettings(
        velocity_up, acceleration_up, deceleration_up, cornering_up,
        velocity_down, acceleration_down, deceleration_down, cornering_down,
        height_up, height_down, height_pressure,
        lowering_speed, raising_speed,
    )

DEFAULT_SETTINGS = MotionSettings()

def settings_constraints_up(settings):
    return (
        settings.velocity_up, 
        settings.acceleration_up, settings.deceleration_up, 
        settings.cornering_up
    )

def settings_constraints_down(settings):
    return (
        settings.velocity_down, 
        settings.acceleration_down, settings.deceleration_down, 
        settings.cornering_down
    )



def raise_pen(*args, **kwargs):
    return lambda s: CommandsBuilder(s).raise_pen(*args, **kwargs)

def lower_pen(*args, **kwargs):
    return lambda s: CommandsBuilder(s).lower_pen(*args, **kwargs)

def walk_path(*args, **kwargs):
    return lambda s: CommandsBuilder(s).walk_path(*args, **kwargs)

def draw_path(*args, **kwargs):
    return lambda s: CommandsBuilder(s).draw_path(*args, **kwargs)

def brush_path(*args, **kwargs):
    return lambda s: CommandsBuilder(s).brush_path(*args, **kwargs)



def pen_height_to_angle(h):
    h = clamp(h, 0,1)
    return math.asin(h*2-1)/math.pi + .5

def angle_to_pen_height(a):
    a = clamp(a, 0,1)
    return (math.sin(a*math.pi-math.pi/2)+1)/2



class CommandsBuilder(object):
    def __init__(self, initial_state=None):
        if initial_state:
            self.microstepping = initial_state.microstepping
            self.pen_height = initial_state.pen_height
            self.pen_down = initial_state.pen_down
            self.pen_position = initial_state.pen_position
        else:
            self.microstepping = 16
            self.pen_height = 1
            self.pen_down = False
            self.pen_position = 0j

        self.steps_err = 0j

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
    def min_velocity(self):
        return MIN_MOTOR_RATE / self.steps_per_mm / SQRT2    # mm/s

    @property
    def max_velocity(self):
        return MAX_MOTOR_RATE / self.steps_per_mm / SQRT2    # mm/s

    @property
    def min_acceleration(self):
        return MIN_ACCELERATION    # mm/s/s
    
    @property
    def max_acceleration(self):
        return MAX_ACCELERATIONS[self.microstepping]    # mm/s/s


    def motionplanning_constraints(self, velocity, accel, decel, cornering):
        v_max = lerp_clamp(velocity, 0,1, self.min_velocity, self.max_velocity)
        accel = lerp_clamp(accel, 0,1, self.min_acceleration, self.max_acceleration)
        decel = lerp_clamp(decel, 0,1, self.min_acceleration, self.max_acceleration)
        cornering = lerp_clamp(cornering, 0,1, MIN_CORNERING, MAX_CORNERING)
        return motionplanning.Constraints(
            v_max = v_max,
            accel = accel, decel = decel,
            cornering_tolerance = cornering
        )


    def enable_motors(self):
        yield enable_motors_command(self.microstepping)

    def disable_motors(self):
        yield disable_motors_command()



    def raise_pen(self, height, duration=None, speed=None, delay=None):
        return self._move_pen(height, False, duration=duration, speed=speed, delay=delay)

    def lower_pen(self, height, duration=None, speed=None, delay=None):
        return self._move_pen(height, True, duration=duration, speed=speed, delay=delay)

    def _move_pen(self, height, pen_down, duration=None, speed=None, delay=None):
        delta = abs(height - self.pen_height)

        if delta > 1e-4:
            if speed:
                duration = delta / speed
            elif duration:
                speed = delta / duration
            else:
                raise ValueError('must provide duration or speed')
            
            if delay == None:
                delay = duration

            yield from SC_commands(pen_down, height, speed)
            yield ('SP', 0 if pen_down else 1, delay*1000)

            self.pen_height = height
            self.pen_down = pen_down
        else:
            if delay:
                yield 'SM',0,0,delay



    def walk_path(self, path, settings=DEFAULT_SETTINGS):
        constraints = self.motionplanning_constraints(*settings_constraints_up(settings))

        plines = path_to_polylines(path, start=self.pen_position)

        yield from self.enable_motors()

        for pline in plines:
            plan = plan_trajectory(chain([self.pen_position], pline), constraints)
            yield from self._do_planned_trajectory(plan)


    def draw_path(self, path, settings=DEFAULT_SETTINGS):
        constraints_up = self.motionplanning_constraints(*settings_constraints_up(settings))
        constraints_down = self.motionplanning_constraints(*settings_constraints_down(settings))
        lowering_speed = lerp_clamp(settings.lowering_speed, 0,1, MIN_PEN_SPEED, MAX_PEN_SPEED)
        raising_speed = lerp_clamp(settings.raising_speed, 0,1, MIN_PEN_SPEED, MAX_PEN_SPEED)
 
        plines = path_to_polylines(path, start=self.pen_position)

        yield from self.enable_motors()

        for pline in plines:
            yield from self.raise_pen(settings.height_up, speed=raising_speed)
            plan_up = plan_trajectory([self.pen_position, pline[0]], constraints_up)
            yield from self._do_planned_trajectory(plan_up)

            if len(pline)>1:
                yield from self.lower_pen(settings.height_down, speed=lowering_speed)
                plan_down = plan_trajectory(pline, constraints_down)
                yield from self._do_planned_trajectory(plan_down)
                yield from self.raise_pen(settings.height_up, speed=raising_speed)


    def brush_path(self, path, pressure, settings=DEFAULT_SETTINGS):
        constraints_up = self.motionplanning_constraints(*settings_constraints_up(settings))
        constraints_down = self.motionplanning_constraints(*settings_constraints_down(settings))
        lowering_speed = lerp_clamp(settings.lowering_speed, 0,1, MIN_PEN_SPEED, MAX_PEN_SPEED)
        raising_speed = lerp_clamp(settings.raising_speed, 0,1, MIN_PEN_SPEED, MAX_PEN_SPEED)
        
        plines = list(path_to_polylines(path, start=self.pen_position))
        pline_lens = [np.sum(np.abs(np.ediff1d(pline))) for pline in plines]
        total_len = sum(pline_lens)

        yield from self.enable_motors()

        start_len = 0
        for pline,pline_len in zip(plines, pline_lens):
            rel_start_len = start_len / total_len
            rel_pline_len = pline_len / total_len
            sub_pressure = [((l-rel_start_len)*total_len, lerp(p, 0,1, settings.height_down, settings.height_pressure)) for l,p in pressure]
            start_len += pline_len

            plan_up = plan_trajectory([self.pen_position, pline[0]], constraints_up)
            plan_down = plan_trajectory(pline, constraints_down, sub_pressure)
            
            yield from self._do_planned_trajectory(plan_up)
            if plan_down.z0 == None:
                yield from self.lower_pen(settings.height_down, speed=lowering_speed)

            yield from self._do_planned_trajectory(plan_down)
            yield from self.raise_pen(settings.height_up, speed=raising_speed)


    def _do_planned_trajectory(self, plan):
        z_moves = plan.z_moves if plan.z_moves else {}

        speed_factor = self.steps_per_mm * SQRT2 # to convert speed in steps/s

        if plan.z0 != None:
            yield from self.lower_pen(plan.z0, speed=2)

        for i,((t0,v0,xy0),(t1,v1,xy1)) in enumerate(pairwise(plan.xy_targets)):
            z_move = z_moves.get(i)
            if z_move:
                z,dt = z_move
                yield from self.lower_pen(z, duration=dt, delay=0)

            if v0 or v1:
                delta = self.project(xy1) - self.project(xy0) # displacement in steps
                self.steps_err,delta = croundf(delta + self.steps_err)
                di,dj = delta.real, delta.imag
                if di or dj:
                    yield LM_command(di, dj, v0*speed_factor, v1*speed_factor)
                    self.pen_position += self.unproject(complex(di,dj))




PlannedTrajectory = namedtuple('PlannedTrajectory', 'xy_targets z0 z_moves')

def plan_trajectory(pline, constraints, pressure=None):

    xys = np.fromiter(uniq(pline), np.complex128)
    if len(xys)<2:
        return PlannedTrajectory([], None, {})

    vectors = np.ediff1d(xys)
    norms = np.abs(vectors)
    norms_cumsum = np.hstack([0, np.cumsum(norms)])

    profile = list(motionplanning.velocity_profile(constraints, vectors/norms, norms))
    
    if not profile:
        return PlannedTrajectory([], None, {})
    
    prefered_timestep = MIN_TIMESTEP if pressure else PREFERED_TIMESTEP
    profile = list(motionplanning.resample_profile(profile, MIN_TIMESTEP, prefered_timestep))
    
    ts,vs,ds = zip(*profile)
    xys = np.interp(ds, norms_cumsum, xys)
    
    if pressure:
        pressure_ds,pressure_zs = zip(*pressure)

        zs = np.interp(ds, pressure_ds, pressure_zs)
        z0 = zs[0]

        mask = rdp.rdp(np.array([ds,zs]).T, algo='iter', return_mask=True, epsilon=0.001)
        z_dts = np.ediff1d(np.array(ts)[mask])
        z_zs = np.array(zs)[mask]
        z_dzs = np.ediff1d(z_zs)
        z_moves = {i:(z_zs[j+1], z_dts[j]) for j,i in enumerate(np.where(mask)[0][:-1]) if z_dzs[j]}
    else:
        z0 = None
        z_moves = {}

    return PlannedTrajectory(list(zip(ts,vs,xys)), z0, z_moves)



def path_to_polylines(path, start=0j):
    if isinstance(path, list):
        line = [p if isinstance(p, complex) else complex(*p[:2]) for p in path]
        return [line]
    else:
        return svg_path_to_polylines(path, start=start)
