import math
from math import sqrt
from collections import namedtuple
import itertools
import logging
import numpy as np


logger = logging.getLogger(__name__)


Constraints = namedtuple('Constraints', 'v_max accel decel cornering_tolerance')

def velocity_profile(vectors, norms, constraints):
    """Compute a velocity profile for a trajectory.

    :param vectors: numpy array of displacement vectors
    :param norms: numpy array of vector norms if `vectors` parameter is already unit vectors
    :param constraints: `Constraints` namedtuple

    :return: list of (time, velocity, distance) tuples
    """
    if not isinstance(norms, np.ndarray):
        norms = np.abs(vectors)
        normed_vectors = vectors/norms
    else:
        normed_vectors = vectors

    return _velocity_profile(
        normed_vectors, norms,
        constraints.v_max, constraints.accel, constraints.decel,
        constraints.cornering_tolerance
    )


def _velocity_profile(normed_vectors, norms, v_max, accel, decel, cornering_tolerance):

    limits = compute_velocity_limits(normed_vectors, norms, v_max, accel, decel, cornering_tolerance)
    limits = np.clip(limits, 0, v_max)

    ds = np.hstack((0, np.cumsum(norms)))
    vs = np.minimum.reduce(limits)

    t0 = 0
    v0 = 0
    for (d1,v1),(d2,v2) in pairwise(zip(ds,vs)):
        tvds = base_velocity_profile(d2-d1, min(v1,v0), v2, v_max, accel, decel)
        if tvds:
            for t,v,d in tvds:
                yield t0+t, v, d1+d
            t0 += tvds[-1][0]
            v0 = tvds[-1][1]


def interpolate_velocity_profile(profile, timestep):

    ts,vs,ds = zip(*profile)

    t0,t1 = min(ts), max(ts)
    d1 = max(ds)

    if t1-t0 > timestep:
        ts2 = np.arange(t0, t1+timestep, timestep)
        vs2 = np.interp(ts2, ts, vs)
        for t,v in zip(ts2,vs2):
            yield t, v, min(integrate(ts, vs, 0, t), d1)
    else:
        yield from zip(ts,vs,ds)



def compute_velocity_limits(normed_vectors, norms, v_max, accel, decel, cornering_tolerance=1):
    """Compute maximum velocity allowed at each point so that...
        - we don't corner too fast
        - we don't try and accelerate faster than we can
        - we don't try and decelerate faster than we can

    :param normed_vectors: numpy array of normed displacement vectors
    :param norms: numpy array of original displacement distances
    :param v_max: maximum velocity constraint
    :param accel: acceleration rate constraint
    :param decel: deceleration rate constraint
    :param cornering_tolerance: cornering tolerance constraint

    :return: tuple of 3 numpy arrays for cornering, acceleration, and deceleration limits
    """
    n = len(normed_vectors) + 1 # so that numpy can prealocate in `fromiter` calls

    cornering_limits = np.fromiter(
        cornering_max_velocities(normed_vectors, accel, cornering_tolerance),
        np.float64, n
    )

    return (
        cornering_limits,
        np.fromiter(acceleration_max_velocities(norms, cornering_limits, accel), np.float64, n),
        np.fromiter(deceleration_max_velocities(norms, cornering_limits, decel), np.float64, n),
    )


def acceleration_max_velocities(norms, target_vs, accel, v0=0):
    """Compute reachable final velocities while conforming to given acceleration rate."""
    yield v0
    for d,target in zip(norms, target_vs[1:]):
        v = min(target, sqrt(2*accel*d + v0**2))
        yield v
        v0 = v


def deceleration_max_velocities(norms, target_vs, decel, v0=0):
    """Compute requiried initial velocities to conform to given deceleration rate."""
    return reversed(list(acceleration_max_velocities(
        norms[::-1], target_vs[::-1],
        decel, v0
    )))


def cornering_max_velocities(normed_vectors, max_accel, tolerance, epsilon=1e-8):
    """Compute cornering velocities.

    see https://onehossshay.wordpress.com/2011/09/24/improving_grbl_cornering_algorithm/
    """

    yield math.inf # no angle at first point

    for u,v in pairwise(normed_vectors):
        dot = u.real*v.real + u.imag*v.imag
        cos_angle = -dot
        sin_half_angle = sqrt((1-cos_angle)/2)
        if 1-sin_half_angle > epsilon: # avoid division by zero
            r = tolerance * sin_half_angle / (1-sin_half_angle)
            yield sqrt(max_accel * r)
        else:
            yield math.inf

    yield math.inf # no angle at last point


def base_velocity_profile(d, v0, v1, v_max, accel, decel):
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

    accel = abs(accel)
    decel = abs(decel)

    if v0 > v_max: raise ValueError('v0 > v_max')
    if v1 > v_max: raise ValueError('v1 > v_max')
    if v0 < 0: raise ValueError('v0 < 0')
    if v1 < 0: raise ValueError('v1 < 0')
    if v_max <= 0: raise ValueError('v_max <= 0')
    if accel <= 0: raise ValueError('accel == 0')
    if decel <= 0: raise ValueError('decel == 0')


    # ideally we'd like to be able to accelerate to v_max and cruise
    # before decelerating to v1

    # time then distance to/from v_max
    t_accel = (v_max - v0) / accel
    t_decel = (v_max - v1) / decel

    d_accel = (v0 * t_accel) + (accel/2 * t_accel**2)
    d_decel = (v1 * t_decel) + (decel/2 * t_decel**2)

    d_cruise = d - (d_accel + d_decel)

    # positive cruising distance means we can have a trapezoid profile
    if d_cruise >= 0 :
        t_cruise = d_cruise / v_max

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

    d_accel = v0 * t_accel + accel/2 * t_accel**2
    d_decel = v1 * t_decel + decel/2 * t_decel**2

    # if that distance is positive we can have a triangle profile
    if d_accel > 0 and d_accel < d:
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

    if abs(v_end - v1) < 1e-12:
        logger.debug('linear, t=%s', t)
    else:
        logger.debug('limited linear, t=%s, v1\'=%s', t, v_end)

    return [
        (0, v0   , 0),
        (t, v_end, d),
    ]





def integrate(xs, ys, x0, x1):
    """Integrates a sampled function."""

    def final_xs():
        yield x0
        yield from itertools.takewhile(lambda x:x <  x1,
                   itertools.dropwhile(lambda x:x <= x0, xs))
        yield x1

    xs2 = list(final_xs())
    ys2 = np.interp(xs2, xs, ys)

    return sum((xb-xa)*(ya+yb)/2 for (xa,ya),(xb,yb) in pairwise(zip(xs2,ys2)))


def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)
