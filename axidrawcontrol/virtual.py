import os, sys
import time, datetime
import subprocess
import threading
import serial
import tempfile
import contextlib
import logging
import re


from .ebb import format_commands
from .simulation import AxidrawState


@contextlib.contextmanager
def socat_serial(port1, port2=None):
    if not port2:
        _, port2 = tempfile.mkstemp()

    cmd = ['/usr/bin/socat', '-d','-d',
           'PTY,link=%s,raw,echo=0' % port1,
           'PTY,link=%s,raw,echo=0' % port2]
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    serialport = None
    for i in range(16):
        try:
            serialport = serial.Serial(port2, baudrate=9600, rtscts=True, dsrdtr=True, timeout=.5)
            break
        except serial.serialutil.SerialException as e:
            time.sleep(.1)
    yield serialport

    time.sleep(.5) #TODO find better solution?

    process.terminate()
    try:
        os.remove(port2)
    except FileNotFoundError:
        pass # happens if we reach here before transmitting anything



def serial_read_chunks(serialport, stop_event=None):
    while True:
        try:
            chunk = serialport.read()
            while serialport.in_waiting > 0:
                chunk += serialport.read()
            yield chunk
            if stop_event and stop_event.is_set():
                return
        except serial.serialutil.SerialException:
            if stop_event and stop_event.is_set():
                return
            else:
                raise




def serial_read_batched_commands(serialport, stop_event=None):
    remaining = b''
    for chunk in serial_read_chunks(serialport, stop_event=stop_event):
        lines = (remaining+chunk).split(b'\r')
        if lines[0]:
            yield lines[:-1]
            remaining = lines[-1]



if __name__ == '__main__':
    import argparse
    import threading

    argparser = argparse.ArgumentParser()
    argparser.add_argument('port', metavar='VIRTUAL-PORT')
    argparser.add_argument('--real-time', type=float, default=0)
    argparser.add_argument('--end-on-EM0', action='store_true')
    argparser.add_argument('--exit-on-EM0', action='store_true')

    args = argparser.parse_args()

    logging.basicConfig(level=logging.DEBUG)
    
    if sys.stdout.isatty():
        BOLD = '\u001b[1m'
        DIM = '\u001b[2m'
        RESET = '\u001b[0m'
    else:
        BOLD = DIM = RESET = ''


    def loop(stop_event=None):
        sim = AxidrawState()
        dump_file = None

        logging.info('virtual Axidraw listening on port %s' % args.port)
        with socat_serial(args.port) as serialport:
            prev_timer_time = 0
            prev_clock_time = 0
            batched = False

            for serial_commands in serial_read_batched_commands(serialport, stop_event):
                simulated = sim.simulate(serial_commands)

                if not dump_file:
                    session_id = datetime.datetime.strftime(datetime.datetime.now(), '%Y%m%d-%H%M%S%f')
                    path = args.port + '_' + session_id + '.ebb'
                    print('dumping to %s' % path)
                    dump_file = open(path, 'wb')

                batched |= len(serial_commands) > 1
                if batched:
                    print()
                    dump_file.write(b'\r')

                for command, response, state in simulated:
                    serialport.write(response)
                    serialport.flush()

                    clock_time = time.time()
                    timer_time = state.timer_time
                    timer_dt = timer_time - prev_timer_time
                    clock_dt = clock_time - prev_clock_time
                    delay = timer_dt - clock_dt
                    if delay > 0:
                        time.sleep(delay * args.real_time)

                    line = re.sub(r'^([A-Za-z0-9]+)', BOLD+r'\1'+RESET, command.decode().ljust(64))
                    if response:
                        line += ' ' + DIM + '# ' + repr(response.decode())[1:-1] + RESET
                    print(line)

                    dump_file.write(command.strip()+b'\r')

                    if command == b'EM,0,0':
                        dump_file.flush()

                        if args.end_on_EM0:
                            dump_file.close()
                            dump_file = None

                        if args.exit_on_EM0:
                            dump_file.close()
                            exit()

                    prev_timer_time = timer_time
                    prev_clock_time = clock_time


    stop_event = threading.Event()
    th = threading.Thread(target=loop, args=(stop_event,))
    try:
        th.start()
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        th.join()
