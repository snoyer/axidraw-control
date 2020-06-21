from math import sqrt
from collections import namedtuple
from itertools import chain, islice
import bisect
import logging
import numpy as np

from .util.misc import pairwise

logger = logging.getLogger(__name__)


Constraints = namedtuple('Constraints', 'v_max accel decel cornering_tolerance')

Waypoint = namedtuple('Waypoint', 't v d')



def velocity_profile(constraints, vectors, norms=None):
    """Compute a velocity profile for a trajectory.

    :param constraints: `Constraints` namedtuple
    :param vectors: numpy array of displacement vectors
    :param norms: numpy array of displacement distances if `vectors` are unit vectors

    :return: list of Waypoint namedtuples (`(time, velocity, distance)`)
    """
    if isinstance(norms, np.ndarray):
        unit_vectors = vectors
        vector_norms = norms
    else:
        vector_norms = np.abs(vectors)
        unit_vectors = vectors/vector_norms

    return flatten_blocks(_velocity_profile(
        unit_vectors, vector_norms,
        constraints.v_max, constraints.accel, constraints.decel, constraints.cornering_tolerance
    ))



def _velocity_profile(unit_vectors, vector_norms, v_max, accel, decel, cornering_tolerance):

    limits = compute_velocity_limits(unit_vectors, vector_norms, v_max, accel, decel, cornering_tolerance)

    if isinstance(v_max, np.ndarray):
        limits = np.vstack((limits, v_max))
    else:
        limits = np.clip(limits, 0, v_max)

    ds = np.hstack((0, np.cumsum(vector_norms)))
    vs = np.minimum.reduce(limits)

    t0,v0,d0 = 0,0,0
    for (d1,v1),(d2,v2) in pairwise(zip(ds,vs)):
        block = [Waypoint(t+t0,v,d+d0) for t,v,d in base_velocity_profile(d2-d1, min(v1,v0), v2, v_max, accel, decel)]
        if block:
            yield block
            t0,v0,d0 = block[-1]
        else:
            yield []



def compute_velocity_limits(unit_vectors, vector_norms, v_max, accel, decel, cornering_tolerance=1):
    """Compute maximum velocity allowed at each point so that...
        - we don't corner too fast
        - we don't try and accelerate faster than we can
        - we don't try and decelerate faster than we can

    :param unit_vectors: numpy array of normed displacement vectors
    :param vector_norms: numpy array of original displacement distances
    :param v_max: maximum velocity constraint
    :param accel: acceleration rate constraint
    :param decel: deceleration rate constraint
    :param cornering_tolerance: cornering tolerance constraint

    :return: tuple of 3 numpy arrays for cornering, acceleration, and deceleration limits
    """
    n = len(unit_vectors) + 1 # so that numpy can prealocate in `fromiter` calls

    cornering_limits = np.fromiter(
        cornering_max_velocities(unit_vectors, accel, cornering_tolerance),
        np.float64, n
    )

    return (
        cornering_limits,
        np.fromiter(acceleration_max_velocities(vector_norms, cornering_limits, accel), np.float64, n),
        np.fromiter(deceleration_max_velocities(vector_norms, cornering_limits, decel), np.float64, n),
    )



def acceleration_max_velocities(norms, target_vs, accel, v0=0):
    """Compute reachable final velocities while conforming to given acceleration rate."""
    yield v0
    for d,target in zip(norms, islice(target_vs,1,None)):
        v = min(target, sqrt(2*accel*d + v0**2))
        yield v
        v0 = v


def deceleration_max_velocities(norms, target_vs, decel, v0=0):
    """Compute requiried initial velocities to conform to given deceleration rate."""
    return reversed(list(acceleration_max_velocities(
        reversed(norms), reversed(target_vs),
        decel, v0
    )))



def cornering_max_velocities(unit_vectors, max_accel, tolerance, epsilon=1e-8, infinity=float('inf')):
    """Compute cornering velocities.

    see https://onehossshay.wordpress.com/2011/09/24/improving_grbl_cornering_algorithm/
    """
    yield infinity # no angle at first point

    for u,v in pairwise(unit_vectors):
        dot = u.real*v.real + u.imag*v.imag
        cos_angle = -dot
        sq_sin_half_angle = (1-cos_angle)/2
        sin_half_angle = sqrt(sq_sin_half_angle) if sq_sin_half_angle > 0 else 0
        if 1-sin_half_angle > epsilon: # avoid division by zero
            r = tolerance * sin_half_angle / (1-sin_half_angle)
            yield sqrt(max_accel * r)
        else:
            yield infinity

    yield infinity # no angle at last point



def base_velocity_profile(d, v0, v1, v_max, accel, decel, t_epsilon=1e-8):
    """Compute a constant acceleration velocity profile for a single move over distance `d`.

    :param d: total distance to cover (unit d)
    :param v0: initial velocity (unit d/t)
    :param v1: final velocity (unit d/t)
    :param v_max: maximum allowed velocity (unit d/t)
    :param accel: constant acceleration rate (unit d/t/t)
    :param decel: constant deceleration rate (unit d/t/t)

    :return: list of (time, velocity, distance) tuples
    """

    if d==0:
        return []

    d = float(d)
    v0 = float(v0)
    v1 = float(v1)
    v_max = float(v_max)
    accel = float(accel)
    decel = float(decel)

    if d <= 0: raise ValueError('d <= 0')
    if v0 > v_max: raise ValueError('v0 > v_max')
    if v1 > v_max: raise ValueError('v1 > v_max')
    if v0 < 0: raise ValueError('v0 < 0')
    if v1 < 0: raise ValueError('v1 < 0')
    if v_max <= 0: raise ValueError('v_max <= 0')
    if accel <= 0: raise ValueError('accel <= 0')
    if decel <= 0: raise ValueError('decel <= 0')


    # if we're going to and from v_max we can just keep going
    if v1 == v0 == v_max:
        t = d / v0
        logger.debug('constant, t=%s', t)
        return [
            (0, v0, 0),
            (t, v1, d),
        ]


    # ideally we'd like to be able to accelerate to v_max and cruise
    # before decelerating to v1

    # time then distance to/from v_max
    t_accel = (v_max - v0) / accel
    t_decel = (v_max - v1) / decel

    d_accel = (v0 * t_accel) + (accel/2 * t_accel**2)
    d_decel = (v1 * t_decel) + (decel/2 * t_decel**2)

    d_cruise = d - (d_accel + d_decel)
    t_cruise = d_cruise / v_max

    # positive cruising distance means we can have a trapezoid profile
    if t_cruise >= t_epsilon :

        logger.debug('trapezoid, t=%s+%s+%s', t_accel, t_cruise, t_decel)
        return [
            (0                           , v0   , 0                 ),
            (t_accel                     , v_max, d_accel           ),
            (t_accel + t_cruise          , v_max, d_accel + d_cruise),
            (t_accel + t_cruise + t_decel, v1   , d                 ),
        ]


    # at this point we know we can't reach v_max,
    # can we still somewhat accelerate from v0 and immediately decelerate to v1?

    accel_sq, decel_sq = accel*accel, decel*decel
    root = sqrt((accel_sq + accel*decel)*v1**2 + 2*d*decel*accel_sq +
                (decel_sq + accel*decel)*v0**2 + 2*d*accel*decel_sq )
    t_accel = ((-decel-accel)*v0 + root)/(accel_sq + accel*decel)
    t_decel = -(v1 - v0 - accel*t_accel)/decel


    # if that distance is positive we can have a triangle profile
    if t_accel > t_epsilon and t_decel > t_epsilon:
        d_accel = v0 * t_accel + accel/2 * t_accel**2
        d_decel = v1 * t_decel + decel/2 * t_decel**2
        v_top = v0 + t_accel * accel

        logger.debug('triangle, t=%s+%s', t_accel, t_decel)
        return [
            (0                , v0   , 0      ),
            (t_accel          , v_top, d_accel),
            (t_accel + t_decel, v1   , d      ),
        ]


    # last ressort is to apply constant acceleration from v0
    # until we reach d at a final velocity that may not be v1

    a = accel if v0 <= v1 else -decel
    t = (sqrt(abs(v0**2 + 2*a*d)) - v0) / a
    v_end = v0 + a * t
    if v_end < 0:
        v_end=0

    if abs(v_end - v1) < 1e-12:
        logger.debug('linear, t=%s', t)
    else:
        logger.debug('limited linear, t=%s, v1\'=%s', t, v_end)

    return [
        (0, v0   , 0),
        (t, v_end, d),
    ]



def resample_profile(profile, min_timestep, target_timestep=None):
    """resample a velocity profile to garantee a minimum duration between waypoints."""

    ts,vs,ds = zip(*profile)

    def to_remove(i, epsilon=1e-3):
        should = ts[i] - ts[i-1] < min_timestep \
              or ts[i+1] - ts[i] < min_timestep
        can = vs[i-1] >= epsilon or vs[i+1] >= epsilon
        return should and can

    def priority(i):
        v0,v,v1 = vs[i-1:i+2]
        if v0 >= v < v1 or v0 > v <= v1:
            return 0, v
        elif v0 <= v > v1 or v0 < v >= v1:
            return 1,-v
        else:
            return 2,-v

    def candidates():
        indices = range(1,len(ts)-1)
        return sorted(filter(to_remove, indices), key=priority)


    q = candidates()
    if q:
        ts,vs,ds = map(list, (ts,vs,ds))
        while q:
            while q:
                i = q.pop()
                if to_remove(i):
                    ts.pop(i)
                    vs.pop(i)
                    ds.pop(i)
                    q = [(j-1 if j>i else j) for j in q]
            q = candidates()

        vs = np.array(vs)
        ds = np.array(ds)
        ts = np.cumsum(np.hstack([0, np.ediff1d(ds)/((vs[:-1]+vs[1:])/2)]))



    if target_timestep:
        def new_ts():
            yield ts[0]
            for t0,t1 in pairwise(ts):
                dt = t1 - t0
                if dt > 0:
                    if dt >= 2*target_timestep:
                        off = (dt%target_timestep) / 2
                        yield from t0 + np.arange(target_timestep+off, dt-target_timestep, target_timestep)
                    yield t1

        new_ts = np.fromiter(new_ts(), np.float64)
        vs = np.interp(new_ts, ts, vs)
        ds = ds[0] + np.hstack([0, np.cumsum(np.ediff1d(new_ts) * (vs[:-1]+vs[1:])/2)])
        ts = new_ts

    return [Waypoint(t,v,d) for t,v,d in zip(ts,vs,ds)]



def flatten_blocks(blocks):
    for i,tvds in enumerate(blocks):
        yield from tvds[1 if i else 0:]


def lerp(t, t0, t1, v0, v1):
    tv = (t - t0) / (t1 - t0)
    return v0 + (v1 - v0) * tv