import os
import math
import logging
from itertools import chain

import matplotlib.pyplot as plt
import numpy as np
import scipy.integrate

from axidrawcontrol import motionplanning, axidraw
from axidrawcontrol.simulation import AxidrawState
from axidrawcontrol.util.misc import pairwise


logging.basicConfig(level=logging.DEBUG)



def main():
    svgd = 'm 0,0 c 2.50,0.38 4.16,3.48 4.33,5.19 0.18,1.71 -0.31,3.55 -0.31,3.55 0,0 5.52,-4.52 6,-7.09 -0.11,-2.23 -3.51,-1.99 -4.25,-0.23 -0.53,0.99 0.27,2.22 1.37,2.21'

    settings1 = axidraw.MotionSettings(velocity=1, acceleration=1, cornering=0.5)
    save_figures('example1', svgd, settings1)

    settings2 = axidraw.MotionSettings(velocity=0.5, acceleration=0.5, cornering=0.5)
    save_figures('example2', svgd, settings2)



def save_figures(basename, svgd, settings, scale=1):
    xys = list(chain.from_iterable(axidraw.path_to_polylines(svgd)))
    xys = [(xy-xys[0])*scale for xy in xys]

    make_planning_plot(xys, settings, save_as=basename+'-planning.png')
    make_simulation_plot(xys, settings, save_as=basename+'-simulation.png')




def make_planning_plot(xys, settings, save_as=None):

    XY = np.fromiter(xys, np.complex128, len(xys))
    vectors = np.ediff1d(XY)
    norms = np.abs(vectors)
    normed_vectors = vectors/norms
    ds0 = np.hstack((0, np.cumsum(norms)))


    axi = axidraw.CommandsBuilder()
    constraints = axi.motionplanning_constraints(*axidraw.settings_constraints_down(settings))

    v_max, accel, decel, cornering_tolerance = constraints
    vl0, vl1, vl2 = motionplanning.compute_velocity_limits(
        normed_vectors, norms,
        v_max, accel, decel, cornering_tolerance
    )

    profile_raw = list(motionplanning.velocity_profile(constraints, vectors))
    profile_fix = list(motionplanning.resample_profile(profile_raw, axidraw.MIN_TIMESTEP, axidraw.PREFERED_TIMESTEP))

    ts2,vs2,ds2 = zip(*profile_fix)
    XY2 = np.interp(ds2, ds0,XY)


    fig1 = plt.figure(figsize=(16,8))

    ax0 = plt.subplot2grid((3, 3), (0, 0), colspan=1)
    ax1 = plt.subplot2grid((3, 3), (0, 1), colspan=2)
    ax2 = plt.subplot2grid((3, 3), (1, 0), colspan=3)
    ax3 = plt.subplot2grid((3, 3), (2, 0), colspan=3)

    fig1.patch.set_alpha(0)
    ax0.patch.set_alpha(0.75)
    ax1.patch.set_alpha(0.75)
    ax2.patch.set_alpha(0.75)
    ax3.patch.set_alpha(0.75)

    ax0.plot(np.real(XY),np.imag(XY), '-o', color='grey', mew=0, ms=4, label='target')
    ax0.plot(np.real(XY2),np.imag(XY2), 'k-', mew=0, ms=4, label='planned')
    _start, = ax0.plot([XY[0].real],[XY[0].imag], 'k*', ms=9, mew=1, mfc='w', label='start')
    ax0.axis('equal')
    ax0.set_title('input trajectory')
    ax0.set_xlabel('x (mm)')
    ax0.set_ylabel('y (mm)')
    ax0.invert_yaxis()
    ax0.legend()

    for i in range(len(XY)):
        ax1.axvline(x=i, ls='--', lw=.1, color='k')

    ax1.axhline(y=v_max, ls='--', lw=.25, color='k')
    ax1.plot(range(len(vl0)), vl1, 'C0--^', label='acceleration', mfc='w', mew=1)
    ax1.plot(range(len(vl0)), vl2, 'C1--v', label='deceleration', mfc='w', mew=1)
    ax1.plot(range(len(vl0)), vl0, 'C2--o', label='cornering', mfc='w', mew=1, ms=4)
    ax1.set_title('limit velocities')
    ax1.set_xlabel('points')
    ax1.set_ylabel('velocity (mm/s)')
    ax1.invert_yaxis()
    handles, labels = ax1.get_legend_handles_labels()
    ax1.legend(handles, labels)
    ax1.set_ylim(-10, max(np.max(vl1),np.max(vl1))*1.25)


    def profile_plot(ax2, profile, title=None):
        ts,vs,ds = zip(*profile)

        for t0,t1 in merge_segments((t0,t1) for t0,t1 in pairwise(ts) if t1-t0 < axidraw.MIN_TIMESTEP-1e-12):
            ax2.axvspan(t0,t1, alpha=0.05, color='red')

        for i,t in enumerate(ts):
            ax2.axvline(x=t, ls='--', lw=.1, color='k')

        ax2.axhline(y=v_max, ls='--', lw=.25, color='k')

        _v, = ax2.plot(ts, vs, '-', color='k', label='velocity')
        ax2b = ax2.twinx()

        ts2 = np.linspace(ts[0],ts[-1], 512)
        vs2 = np.interp(ts2, ts, vs)
        _d, = ax2b.plot(ts2, scipy.integrate.cumtrapz(vs2, x=ts2, initial=0), 'C3', label='distance')

        if title:
            ax2.set_title(title)
        ax2.set_xlabel('time (s)')
        ax2.set_ylabel('velocity (mm/s)')
        ax2b.set_ylabel('distance (mm)')
        ax2.legend(handles=[_v,_d])

    profile_plot(ax2, profile_raw, 'constant acceleration profile')
    ts,vs,ds = zip(*profile_raw)
    lvmin = np.minimum.reduce([vl0,vl1,vl2])
    points_ts = np.interp(ds0, ds, ts)
    ax2.plot(points_ts, np.where(vl2==lvmin, vl2, np.nan), 'kv', label='deceleration', mfc='w', mew=.5)
    ax2.plot(points_ts, np.where(vl1==lvmin, vl1, np.nan), 'k^', label='acceleration', mfc='w', mew=.5)
    ax2.plot(points_ts, np.where(vl0==lvmin, vl0, np.nan), 'ko', label='cornering', mfc='w', mew=.5, ms=4)

    profile_plot(ax3, profile_fix, 'resampled profile')

    axs = [ax2,ax3]
    x0s,x1s = zip(*[ax.get_xlim() for ax in axs])
    for ax in axs:
        ax.set_xlim(min(*x0s),max(*x1s))

    plt.tight_layout()

    if save_as:
        fig1.savefig(save_as)
    else:
        plt.show()



def segments_overlap(a,b):
    s1,e1 = a
    s2,e2 = b
    return s2<=e1 and e2>=s1

def merge_segments(segments):
    segments = sorted(segments)

    start,end = None,None
    
    for s,e in segments:
        if (start,end) == (None,None):
            start,end = s,e
        else:
            if segments_overlap((start,end),(s,e)):
                end = max(end, e)
            else:
                yield start,end
                start,end = s,e
    
    yield start,end



def make_simulation_plot(xys, settings, save_as=None):
    def simulated():
        sim = AxidrawState()
        axi = axidraw.CommandsBuilder()
        for s in sim.simulate(axi.walk_path(xys, settings)):
            yield s.state.timer_time, s.state.pen_position


    XY0 = np.fromiter(xys, np.complex128, len(xys))
    
    T,XY = map(np.array, zip(*simulated()))

    dXY = np.ediff1d(XY)
    nXY = np.abs(dXY)
    V = np.hstack((0, nXY/np.ediff1d(T)))
    D = np.hstack((0, np.cumsum(nXY)))


    fig1 = plt.figure(figsize=(16,8))
    ax0 = plt.subplot2grid((3, 1), (1, 0), rowspan=2)
    ax1 = plt.subplot2grid((3, 1), (0, 0))
    ax1b = ax1.twinx()


    fig1.patch.set_alpha(0)
    ax0.patch.set_alpha(0.75)
    ax1.patch.set_alpha(0.75)

    ax0.set_title('simulated output trajectory')
    ax0.set_xlabel('x (mm)')
    ax0.set_ylabel('y (mm)')
    ax0.axis('equal')
    ax0.invert_yaxis()

    ax0.plot(np.real(XY0), np.imag(XY0), '-o', color='grey', mew=0, ms=4, label='target')
    ax0.plot(np.real(XY), np.imag(XY), 'k-', mew=0, ms=4, label='simulated')
    ax0.plot([XY[0].real],[XY[0].imag], 'k*', ms=9, mew=1, mfc='w', label='start')

    ax0.legend()


    axi = axidraw.CommandsBuilder()
    constraints = axi.motionplanning_constraints(*axidraw.settings_constraints_down(settings))
    ax1.axhline(y=constraints.v_max, ls='--', lw=.25, color='k')
    for i,t in enumerate(T):
        ax1.axvline(x=t, ls='--', lw=.1, color='k')
    _v, = ax1.plot(T,V, '-', color='k', label='velocity')
    _d, = ax1b.plot(T,D, 'C3', label='distance')

    ax1.set_title('simulated profile')
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('velocity (mm/s)')
    ax1b.set_ylabel('distance (mm)')
    ax1.legend(handles=[_v,_d])

    plt.tight_layout()

    if save_as:
        fig1.savefig(save_as)
    else:
        plt.show()

if __name__ == '__main__':
    main()

