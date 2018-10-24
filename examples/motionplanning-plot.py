
import os
import math
import logging
import itertools

import matplotlib.pyplot as plt
import numpy as np
import scipy.integrate

from axidrawcontrol import motionplanning


logging.basicConfig(level=logging.DEBUG)



def make_plot(xys, constraints, save_as=None):
    v_max, accel, decel, cornering_tolerance = constraints

    XY = np.fromiter((complex(*xy) for xy in xys), np.complex128, len(xys))
    vectors = np.ediff1d(XY)
    norms = np.abs(vectors)
    normed_vectors = vectors/norms
    ds0 = np.hstack((0, np.cumsum(norms)))

    vl0, vl1, vl2 = motionplanning.compute_velocity_limits(
        normed_vectors, norms,
        v_max, accel, decel, cornering_tolerance
    )

    profile = list(motionplanning.velocity_profile(constraints, vectors))
    ts,vs,ds = zip(*motionplanning.concat_blocks(profile))



    fig1 = plt.figure(figsize=(16,6))


    ax0 = plt.subplot2grid((2, 3), (0, 0), colspan=1)
    ax1 = plt.subplot2grid((2, 3), (0, 1), colspan=2)
    ax2 = plt.subplot2grid((2, 3), (1, 0), colspan=3)

    fig1.patch.set_alpha(0)
    ax0.patch.set_alpha(0.75)
    ax1.patch.set_alpha(0.75)
    ax2.patch.set_alpha(0.75)

    ax0.plot(*zip(*xys), 'k-o', mew=0, ms=4)
    _start, = ax0.plot(*zip(*xys[:1]), 'k*', ms=9, mew=1, mfc='w', label='start')
    ax0.axis('equal')
    ax0.set_title('input trajectory')
    ax0.set_xlabel('x (mm)')
    ax0.set_ylabel('y (mm)')
    ax0.legend(handles=[_start])


    for i in range(len(xys)):
        ax1.axvline(x=i, ls='--', lw=.1, color='k')

    ax1.axhline(y=v_max, ls='--', lw=.25, color='k')
    ax1.plot(range(len(vl0)), vl1, 'C0--^', label='acceleration', mfc='w', mew=1)
    ax1.plot(range(len(vl0)), vl2, 'C1--v', label='deceleration', mfc='w', mew=1)
    ax1.plot(range(len(vl0)), vl0, 'C2--o', label='cornering', mfc='w', mew=1, ms=4)
    ax1.set_title('computed limit velocities')
    ax1.set_xlabel('points')
    ax1.set_ylabel('velocity (mm/s)')
    handles, labels = ax1.get_legend_handles_labels()
    ax1.legend(handles, labels)
    ax1.set_ylim(-10, max(np.max(vl1),np.max(vl1))*1.25)

    for i,t in enumerate(np.interp(ds0, ds, ts)):
        ax2.axvline(x=t, ls='--', lw=.1, color='k')

    ax2.axhline(y=v_max, ls='--', lw=.25, color='k')

    _v, = ax2.plot(ts, vs, 'k', label='velocity')
    ax2b = ax2.twinx()

    ts2 = np.linspace(ts[0],ts[-1], 512)
    vs2 = np.interp(ts2, ts, vs)
    _d, = ax2b.plot(ts2, scipy.integrate.cumtrapz(vs2, x=ts2, initial=0), 'C3', label='distance')


    lvmin = np.minimum.reduce([vl0,vl1,vl2])
    points_ds = np.interp(ds0, ds, ts)
    ax2.plot(points_ds, np.where(vl2==lvmin, vl2, np.nan), 'kv', label='deceleration', mfc='w', mew=.5)
    ax2.plot(points_ds, np.where(vl1==lvmin, vl1, np.nan), 'k^', label='acceleration', mfc='w', mew=.5)
    ax2.plot(points_ds, np.where(vl0==lvmin, vl0, np.nan), 'ko', label='cornering', mfc='w', mew=.5, ms=4)


    ax2.set_title('final velocity profile')
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('velocity (mm/s)')
    ax2b.set_ylabel('distance (mm)')
    ax2.legend(handles=[_v,_d])


    plt.tight_layout()

    if save_as:
        fig1.savefig(save_as)
    else:
        plt.show()


def read_trajectory(path):
    for line in open(path):
        yield [float(x) for x in line.strip().split(' ')]

xys = list(read_trajectory('trajectory1-small.txt'))
# xys = list(read_trajectory('trajectory1.txt'))
constraints = motionplanning.Constraints(250, 900, 900, 0.65)

make_plot(xys, constraints, save_as='motionplanning-plot.png')
