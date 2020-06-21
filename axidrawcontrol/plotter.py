from collections import namedtuple
from itertools import chain
import sys
import time
import logging

import numpy as np
import rdp

from .simulation import AxidrawState
from .axidraw import CommandsBuilder
from .util.misc import split_iterable, pretty_time_delta, uniq

logger = logging.getLogger(__name__)


SimulatedState = namedtuple('SimulatedState', 'timer_time pen_down pen_position expects_OK')
SimulatedBatch = namedtuple('SimulatedBatch', 'commands start_state end_state travel')


def simulate_and_batch(commands, state=None):
    """Batch a sequence of EBB commands into meaningful batches."""
    if not state:
        state = AxidrawState()

    def break_between(a,b):
        return (a.state.pen_down==False and b.command[:2] == ('SP',0)) \
            or (a.command[:2] == ('SP',1) and b.state.pen_down==False)

    simulated = state.simulate(commands, lambda s:SimulatedState(s.timer_time, s.pen_down, s.pen_position, s.expects_OK))

    for batch in split_iterable(simulated, between=break_between):

        travel = list(uniq(s.state.pen_position for s in batch))

        yield SimulatedBatch(
            [s.command for s in batch],
            batch[ 0].state,
            batch[-1].state,
            travel,
        )


def do_plot(ebb, commands, walk_hull=False, stop_event=None, file=None):
    for stats in do_plot_iter(ebb, commands, walk_hull=walk_hull, stop_event=stop_event):
        if stats:
            task, elapsed, progress, eta = stats
            print_progressbar(task, progress, elapsed, eta, file=file)
        else:
            print('', file=file)


def do_plot_iter(ebb, commands, walk_hull=False, state=None, stop_event=None):
    simulated = []
    try:
        total = len(commands)
    except TypeError:
        total = -1

    n = 0
    t0 = time.time()
    for batch in simulate_and_batch(commands, state=state):
        simulated.append(batch)
        n += len(batch.commands)
        progress = (n/total) if total>=0 else -1
        yield 'simulating', time.time()-t0, progress, -1

    yield None

    button_pressed = ButtonHelper(ebb, .5)


    if walk_hull:
        try:
            import scipy
            travel = list(chain.from_iterable([p for p in batch.travel] for batch in simulated))
            hull = scipy.spatial.ConvexHull(travel)
            hull_path = [hull.points[j] for i,j in enumerate(hull.vertices)]
            
            cb = CommandsBuilder()
            for i in range(5):
                ebb.run(cb.walk_path(hull_path))
                if button_pressed():
                    break
            ebb.run(cb.walk_path([0j]))
        except ImportError:
            logger.warn('could not import scipy, skipping convex hull')


    started_at = time.time()
    paused_for = 0
    
    pause_requested = False

    total_time = simulated[-1].end_state.timer_time
    
    for simulated_batch in simulated:
        if stop_event and stop_event.is_set():
            break

        pause_requested = pause_requested or button_pressed()
        
        if pause_requested and not simulated_batch.start_state.pen_down:
            paused_at = time.time()
            
            yield 'paused', timer_time, progress, eta

            while not button_pressed():
                time.sleep(0.1)

            resumed_at = time.time()
            paused_for += resumed_at - paused_at

            pause_requested = False


        if simulated_batch.end_state.expects_OK:
            actual_commands = chain(['CU,1,0\r'], simulated_batch.commands, ['CU,1,1\r'])
        else:
            actual_commands = chain(['CU,1,0\r'], simulated_batch.commands)

        expected_run_time = simulated_batch.end_state.timer_time - simulated_batch.start_state.timer_time
        ebb.run(actual_commands, timeout= expected_run_time+1)

        
        clock_time = time.time() - started_at
        timer_time = clock_time - paused_for

        progress = simulated_batch.end_state.timer_time / total_time
        eta = timer_time/progress - timer_time

        yield 'drawing', timer_time, progress, eta

    
    while not ''.join(ebb.run('QM')).startswith('QM,0,0,0,0'):
        pass
    
    ebb.run('EM,0,0')

    yield 'done', timer_time, 1, 0





class ButtonHelper(object):
    def __init__(self, ebb, min_delay=.5, reset=True):
        self.ebb = ebb
        self.min_delay = min_delay
        self.button_pressed_at = 0

        if reset:
            self.ebb.run('QB')

    def __call__(self):
        if ''.join(self.ebb.run('QB')).startswith('1'):
            t = time.time()
            if t - self.button_pressed_at >= self.min_delay:
                self.button_pressed_at = t
                self.ebb.run('QB')
                return True
        return False




import shutil
def print_progressbar(task, progress, elapsed, eta, file=None):
    terminal_size = shutil.get_terminal_size((80, 20))

    prefix = task
    if progress >= 0:
        prefix += ' %5.1f%%' % (progress*100)

    suffix = pretty_time_delta(elapsed)
    if eta >0:
        suffix += ' (%s)' % pretty_time_delta(eta)

    prefix = '%20s [' % prefix
    suffix = '] %s' % suffix
    barlen = terminal_size.columns - len(suffix) - len(prefix)
    
    if progress >= 0 :
        n = int(round(progress*barlen))
        bar = '='*n + ' '*(barlen-n)
    else:
        n = int((time.time()*barlen/5)%barlen)
        bar = ' '*n + '=' + ' '*(barlen-n-1)

    if not file:
        file = sys.stdout
    file.write('\r'+prefix+bar+suffix)
    file.flush()

