from collections import namedtuple
from typing import NamedTuple
import math
from itertools import chain, takewhile
import logging

import numpy as np
import rdp

from .ebb import LM_command, SC_commands, enable_motors_command, disable_motors_command
from . import motionplanning
from .util.svg import path_to_polylines as svg_path_to_polylines
from .util.misc import pairwise, lerp, clamp, lerp_clamp, croundf, uniq, parse_value


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


    def enable_motors(self, microstepping):
        self.microstepping = microstepping
        yield enable_motors_command(microstepping)

    def disable_motors(self):
        yield disable_motors_command()

    def delay(self, duration):
        if duration > 1e-3:
            yield 'SM',duration*1000,0,0


    def raise_pen(self, height, duration=None, speed=None, delay=None, extra_delay=0):
        return self._move_pen(height, False, duration=duration, speed=speed, delay=delay, extra_delay=extra_delay)

    def lower_pen(self, height, duration=None, speed=None, delay=None, extra_delay=0):
        return self._move_pen(height, True , duration=duration, speed=speed, delay=delay, extra_delay=extra_delay)

    def _move_pen(self, height, pen_down, duration=None, speed=None, delay=None, extra_delay=0):
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

            yield from SC_commands(pen_down, pen_height_to_angle(height), speed)
            yield ('SP', 0 if pen_down else 1, (delay+extra_delay)*1000)

            self.pen_height = height
            self.pen_down = pen_down
        else:
            if delay:
                yield from self.delay((delay+extra_delay)*1000)



    def walk_path(self, path, settings=None):
        if not settings:
            settings = DEFAULT_SETTINGS
        
        constraints = settings.pen_up.constraints

        plines = path_to_polylines(path, start=self.pen_position)

        yield from self.enable_motors(settings.microstepping)

        for pline in plines:
            plan = plan_trajectory(chain([self.pen_position], pline), constraints)
            yield from self._do_planned_trajectory(plan)


    def draw_path(self, path, settings=None):
        if not settings:
            settings = DEFAULT_SETTINGS
        
        plines = path_to_polylines(path, start=self.pen_position)

        yield from self.enable_motors(settings.microstepping)

        yield from self.raise_pen(settings.pen_up.min_height, speed=settings.raising.speed)
        for pline in plines:
            plan_up = plan_trajectory([self.pen_position, pline[0]], settings.pen_up.constraints)
            yield from self._do_planned_trajectory(plan_up)

            if len(pline)>1:
                yield from self.delay(settings.lowering.pre_delay)
                yield from self.lower_pen(settings.pen_down.max_height, speed=settings.lowering.speed,
                                          extra_delay=settings.lowering.post_delay)
    
                plan_down = plan_trajectory(pline, settings.pen_down.constraints)
                yield from self._do_planned_trajectory(plan_down)
                
                yield from self.delay(settings.raising.pre_delay)
                yield from self.raise_pen(settings.pen_up.min_height, speed=settings.raising.speed,
                                          extra_delay=settings.raising.post_delay)


    def brush_path(self, path, pressure, settings=None):
        if not settings:
            settings = DEFAULT_SETTINGS

        plines = list(path_to_polylines(path, start=self.pen_position))
        pline_lens = [np.sum(np.abs(np.ediff1d(pline))) for pline in plines]
        total_len = sum(pline_lens)

        yield from self.enable_motors(settings.microstepping)

        start_len = 0
        for pline,pline_len in zip(plines, pline_lens):
            rel_start_len = start_len / total_len
            rel_pline_len = pline_len / total_len
            h0 = settings.pen_down.max_height
            h1 = settings.pen_down.min_height
            sub_pressure = [((l-rel_start_len)*total_len, lerp(p, 0,1, h0,h1)) for l,p in pressure]
            start_len += pline_len

            plan_up = plan_trajectory([self.pen_position, pline[0]], settings.pen_up.constraints)
            plan_down = plan_trajectory(pline, settings.pen_down.constraints, sub_pressure)
            
            yield from self._do_planned_trajectory(plan_up)
            if plan_down.z0 == None:
                yield from self.lower_pen(settings.pen_down.max_height, speed=settings.lowering.speed)

            yield from self._do_planned_trajectory(plan_down)
            yield from self.raise_pen(settings.pen_up.min_height, speed=settings.raising.speed)


    def _do_planned_trajectory(self, plan):
        z_moves = plan.z_moves if plan.z_moves else {}

        speed_factor = self.steps_per_mm * SQRT2 # to convert speed in steps/s

        if plan.z0 != None:
            yield from self.lower_pen(plan.z0, speed=2)

        for i,((t0,v0,xy0),(t1,v1,xy1)) in enumerate(pairwise(plan.xy_targets)):
            z_move = z_moves.get(i)
            if z_move:
                z,dt = z_move
                yield from self.lower_pen(pen_height_to_angle(z), duration=dt, delay=0)

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





class MoveSettings(NamedTuple):
    max_velocity: float
    acceleration: float
    deceleration: float
    cornering: float
    min_height: float
    max_height: float

    @property
    def constraints(self):
        return motionplanning.Constraints(
            v_max = self.max_velocity,
            accel = self.acceleration,
            decel = self.deceleration,
            cornering_tolerance = self.cornering
        )


class TransitionSettings(NamedTuple):
    speed: float
    pre_delay: float
    post_delay: float


class MotionSettings(NamedTuple):
    microstepping: int
    pen_up: MoveSettings
    pen_down: MoveSettings
    lowering: TransitionSettings
    raising: TransitionSettings

    def normalized(self):
        microstepping = self.microstepping
        if not microstepping in (16,8,4):
            raise ValueError('microstepping setting must be one of [4,8,16]')

        steps_per_mm = STEPS_PER_MM_AT_1X * microstepping
        min_v = MIN_MOTOR_RATE / steps_per_mm / SQRT2  # mm/s
        max_v = MAX_MOTOR_RATE / steps_per_mm / SQRT2  # mm/s
        min_a = MIN_ACCELERATION                  # mm/s/s
        max_a = MAX_ACCELERATIONS[microstepping]  # mm/s/s

        def fix_MoveSettings(s, default_height):
            return MoveSettings(
                max_velocity = parse_value(s.max_velocity, '50%', [min_v, max_v], 'mm/s'),
                acceleration = parse_value(s.acceleration, '50%', [min_a, max_a], 'mm/s/s'),
                deceleration = parse_value(s.deceleration, '50%', [min_a, max_a], 'mm/s/s'),
                cornering = parse_value(s.cornering, '50%', [MIN_CORNERING,MAX_CORNERING]),
                min_height = parse_value(s.min_height, default_height, [0,1]),
                max_height = parse_value(s.max_height, default_height, [0,1]),
            )

        def fix_TransitionSettings(s):
            return TransitionSettings(
                speed = parse_value(s.speed, 2, [MIN_PEN_SPEED, MAX_PEN_SPEED]),
                pre_delay  = parse_value(s.pre_delay , 0, [0,2], "s"),
                post_delay = parse_value(s.post_delay, 0, [0,2], "s"),
            )

        return MotionSettings(
            microstepping = microstepping,
            pen_up   = fix_MoveSettings(self.pen_up  , default_height=.75),
            pen_down = fix_MoveSettings(self.pen_down, default_height=.25),
            lowering = fix_TransitionSettings(self.lowering),
            raising  = fix_TransitionSettings(self.raising),
        )


def parse_settings(arg=None, **kwargs):
    args_dict = dict(arg) if arg else dict()
    args_dict.update(kwargs)

    def find(*keys, default=None, sep='/'):
        def alt_keys(key):
            yield key
            yield key.replace('-', '_')

        for key in keys:
            for alt_key in alt_keys(key):
                try:
                    v = args_dict
                    for k in alt_key.split(sep):
                        v = v[k]
                    return v
                except (KeyError,TypeError):
                    pass
        return default

    microstepping = kwargs.get('microstepping', 16)
    
    def parse_MoveSettings(d):
        return MoveSettings(
            max_velocity = find(d+'/max-velocity', 'max-velocity', 'velocity'),
            acceleration = find(d+'/acceleration', 'acceleration'),
            deceleration = find(d+'/deceleration', 'deceleration'),
            cornering    = find(d+'/cornering'   , 'cornering'),
            min_height   = find(d+'/min-height'  , d+'/height', 'pen-'+d),
            max_height   = find(d+'/max-height'  , d+'/height', 'pen-'+d),
        )

    def parse_TransitionSettings(d):
        return TransitionSettings(
            speed = find(d+'/speed', d+'-speed'),
            pre_delay  = find(d+'/pre-delay' , d+'-pre-delay' ),
            post_delay = find(d+'/post-delay', d+'-post-delay'),
        )

    return MotionSettings(
        microstepping = microstepping,
        pen_up   = parse_MoveSettings('up'  ),
        pen_down = parse_MoveSettings('down'),
        lowering = parse_TransitionSettings('lowering'),
        raising  = parse_TransitionSettings('raising' ),
    ).normalized()



DEFAULT_SETTINGS = parse_settings()