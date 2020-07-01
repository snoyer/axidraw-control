import re
from collections import Iterable
from types import GeneratorType
import serial
import asyncio, serial_asyncio
import time
import logging


logger = logging.getLogger(__name__)


SERVO_MIN =  7500    # lowest pen position in servo units
SERVO_MAX = 28000    # highest pen position in servo units
SERVO_SPEED_UNIT = (SERVO_MAX - SERVO_MIN) * 24 / 1000
LM_FREQUENCY = 25000    # low-level move motor frequency in Hz
LM_RATE_FACTOR = (2**31)/LM_FREQUENCY    # low-level rate factor


ENCODING = 'ascii'
RESPONSE_PATTERNS = {
    'A' : r'A(,\d\d:\d\d\d\d)*\r',
    'AC': r'OK\r\n',
    'BL': r'OK\r\n',
    'C' : r'OK\r\n',
    'CN': r'OK\r\n',
    'CS': r'OK\r\n',
    'CK': r'(.+=.+\r\n)+OK\r\n',
    'CU': r'OK\r\n',
    'EM': r'OK\r\n',
    'EM': r'OK\r\n',
    'ES': r'\d,\d+,\d+,\d+,\d+\n\rOK\r\n',
    'HM': r'OK\r\n',
    'I' : r'I,\d+,\d+,\d+,\d+,\d+\r\n',
    'LM': r'OK\n\r',
    'MR': r'MR,\d+\r\n',
    'MW': r'OK\r\n',
    'ND': r'OK\r\n',
    'NI': r'OK\r\n',
    'O' : r'OK\r\n',
    'PC': r'OK\r\n',
    'PD': r'OK\r\n',
    'PG': r'OK\r\n',
    'PI': r'PI,\d+\r\n',
    'PO': r'OK\r\n',
    'QB': r'\d\r\nOK\r\n',
    'QC': r'\d+,\d+\r\nOK\r\n',
    'QG': r'[0-9a-f]+\r\n',
    'QL': r'\d+\r\nOK\r\n',
    'QM': r'QM,\d,\d,\d,\d\n\r',
    'QP': r'\d\n\rOK\r\n',
    'QR': r'\d\n\rOK\r\n',
    'QS': r'\d+,\d+\n\rOK\r\n',
    'QT': r'.*\r\nOK\r\n',
    'RB': r'',
    'R' : r'OK\r\n',
    'SC': r'OK\r\n',
    'SE': r'OK\r\n',
    'SL': r'OK\r\n',
    'SM': r'OK\r\n',
    'SN': r'OK\r\n',
    'SP': r'OK\r\n',
    'SR': r'OK\r\n',
    'ST': r'OK\r\n',
    'T' : r'OK\r\n',
    'S2': r'OK\r\n',
    'TP': r'OK\r\n',
    'QN': r'\d+\r\nOK\r\n',
    'V' : r'.+\r\n',
    'XM': r'OK\r\n',
}
ERR_PATTERN = r'!.+\r\n'

def compile_response_patterns():
    def fix_pattern(p, remove_OK=False):
        p = re.sub(r'\\r\\n|\\n\\r', r'(\r\n|\n\r)', p)
        if remove_OK:
            p = p.replace('OK(\r\n|\n\r)', '')
        return f'({p})|({ERR_PATTERN})'

    for k,v in RESPONSE_PATTERNS.items():
        yield k, (
            re.compile(fix_pattern(v, remove_OK=False).encode(ENCODING)),
            re.compile(fix_pattern(v, remove_OK=True ).encode(ENCODING)),
        )
RESPONSE_PATTERNS = dict(compile_response_patterns())



def microstepping_to_stepmode(microstepping):
    if microstepping == 16: return 1
    if microstepping ==  8: return 2
    if microstepping ==  4: return 3
    if microstepping ==  2: return 4
    if microstepping ==  1: return 5
    raise ValueError('microstepping must be one of: 1,2,4,8,16')


def stepmode_to_microstepping(stepmode):
    if stepmode == 1: return 16
    if stepmode == 2: return  8
    if stepmode == 3: return  4
    if stepmode == 4: return  2
    if stepmode == 5: return  1
    raise ValueError('stepmode must be one of: 1,2,3,4,5')



def enable_motors_command(microstepping):
    stepmode = microstepping_to_stepmode(microstepping)
    return ('EM', stepmode, stepmode)


def disable_motors_command():
    return ('EM', 0, 0)


def SC_commands(pen_down, pos, rate):
    sp,sr = (5,12) if pen_down else (4,11)

    if pos > 1: pos = 1
    if pos < 0: pos = 0
    return [
        ('SC', sp, SERVO_MIN + pos*(SERVO_MAX-SERVO_MIN)),
        ('SC', sr, rate * SERVO_SPEED_UNIT), #TODO double check
    ]


def LM_command(di,dj, v0, v1):
    if di or dj:
        l = abs(complex(di,dj)) # displacement norm
        li = abs(di/l) # normalized i component
        lj = abs(dj/l) # normalized j component
        return ('LM', *LM_axis_params(di, v0*li, v1*li),
                      *LM_axis_params(dj, v0*lj, v1*lj))
    raise ValueError('cannot generate LM command for (0,0) steps move')


def LM_axis_params(steps, v0, v1):
    if steps == 0:
        return 0,0,0

    rate0 = round(LM_RATE_FACTOR * v0)
    rate1 = round(LM_RATE_FACTOR * v1)

    duration = 2 * abs(steps) / (v0+v1)
    delta = round((rate1-rate0) / (duration*LM_FREQUENCY))

    return rate0, steps, delta



CMD_SEP_RE = re.compile(r'\s')

def parse_commands(x):
    """Generate proper EBB command tuples."""
    def split(x):
        return CMD_SEP_RE.split(x.strip())

    if isinstance(x, (bytes, bytearray)):
        for cmd in split(x.decode(ENCODING)):
            if cmd:
                yield parse_command(cmd)
    elif isinstance(x, str):
        for cmd in split(x):
            if cmd:
                yield parse_command(cmd)
    elif isinstance(x, tuple):
        yield parse_command(x)
    elif isinstance(x, (GeneratorType, Iterable)):
        for y in x:
            yield from parse_commands(y)
    else:
        raise ValueError("cannot parse commands from %s" % repr(x))


def parse_command(cmd):
    """Convert to proper EBB command tuple."""
    if isinstance(cmd, (bytes, bytearray)):
        split = cmd.strip().decode(ENCODING).split(',')
    elif isinstance(cmd, str):
        split = cmd.strip().split(',')
    elif isinstance(cmd, tuple):
        split = cmd
    else:
        raise ValueError("cannot parse command from %s" % repr(cmd))

    return tuple(_parse_command_values(split))




def serialize_commands(cmds):
    return b''.join(map(serialize_command, cmds))

def serialize_command(cmd):
    if isinstance(cmd, tuple):
        return (','.join(_format_command_values(cmd))+'\r').encode(ENCODING)
    else:
        raise ValueError("cannot serialize command %s" % repr(cmd))




def format_commands(cmds, separator='\n'):
    return separator.join(map(format_command, cmds))

def format_command(cmd):
    return ','.join(_format_command_values(cmd))




def _parse_command_values(values):
    for value in values:
        try:
            yield int(round(float(value)))
        except ValueError:
            yield value

def _format_command_values(values):
    for value in values:
        if isinstance(value, float):
            yield str(int(round(value)))
        else:
            yield str(value)



def read_commands(f):
    sep, empty = b'\r', b''
    remaining = empty
    while True:
        chunk = f.read(512)
        if chunk == empty:
            break
        lines = (remaining+chunk).split(sep)
        if any(lines[:-1]):
            yield from filter(lambda c:c[0], map(parse_command, lines[:-1]))
            remaining = lines[-1]


def check_version_response(response):
    return response.strip().startswith('EBB')


class SerialEbbError(IOError):
    pass





class SerialEbb(object):
    def __init__(self, port=None, connection_timeout=20, command_timeout=1):
        if not port:
            port = find_EBB_port()

        try:
            read_timeout = 0.05
            self.serialport = serial.Serial(port, timeout=read_timeout)
        except OSError as e:
            if e.errno == 16:
                raise SerialEbbError('port `%s` busy' % port)
            else:
                raise SerialEbbError(e.strerror)

        self.command_timeout = command_timeout
        self.expect_OK = True

        version = ''.join(self.run('V')).strip()
        if check_version_response(version):
            logger.info('confirmed EBB (%s) on port %s', version, port)
            self.run(('CU',1, 1 if self.expect_OK else 0))
        else:
            logger.error('could not confirm EBB on port %s', port)




    def run(self, command_or_commands, timeout=0):

        parsed_commands = list(parse_commands(command_or_commands))
        cmds_bytes = serialize_commands(parsed_commands)

        logger.debug('-> %s', repr(cmds_bytes))
        t0 = time.time()

        self.serialport.write(cmds_bytes)
        responses = list(self._read_responses(parsed_commands, timeout))

        t1 = time.time()
        logger.debug('<- %s (%fs)', repr(responses), t1-t0)

        return [response.decode(ENCODING) for response in responses]


    def _read_responses(self, parsed_commands, total_timeout=0):
        readbuffer = self.serialport.read(512)

        giveup_time = time.time() + total_timeout

        for cmd,*args in parsed_commands:
            cmd = cmd.upper()
            if cmd == 'CU' and args[0] == 1:
                self.expect_OK = bool(args[1])
            
            pattern = RESPONSE_PATTERNS[cmd][0 if self.expect_OK else 1]
            
            giveup_time = max(giveup_time, time.time() + self.command_timeout)

            response_bytes = None
            while time.time() < giveup_time:
                m = pattern.search(readbuffer)
                if m:
                    if m.start()!=0:
                        logger.warn('dropped %s', readbuffer[:m.start()])
                    response_bytes = m.group(0)
                    readbuffer = readbuffer[m.end():]
                    break
                else:
                    readbuffer += self.serialport.read(512)

            if response_bytes != None:
                yield response_bytes
            else:
                logger.warn('no response for %s', format_command((cmd,*args)))

        if readbuffer:
            logging.warn('unexpected response: %s (for %s)', readbuffer, format_commands(parsed_commands))


    def close(self):
        if self.serialport:
            logger.info('closing serial port')
            self.serialport.close()
        else:
            logger.warn('no serial port to close')


    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()


    @property
    def port(self):
        return self.serialport.port


    def __str__(self):
        return '<SerialEbb on %s>' % self.port


class AioSerialEbb(object):
    @classmethod
    async def create(cls, port=None, connection_timeout=20, command_timeout=1):
        if not port:
            port = find_EBB_port()

        retry_until = time.time() + connection_timeout
        while True:
            try:
                reader,writer = await serial_asyncio.open_serial_connection(url=port)
                break
            except OSError as e:
                if e.errno == 16:
                    if time.time() <= retry_until:
                        logger.info('port `%s` busy, retrying...', port)
                        await asyncio.sleep(1)
                    else:
                        raise SerialEbbError('port `%s` busy' % port)
                else:
                    if time.time() <= retry_until:
                        logger.info('cannot connect to port `%s`, retrying...', port)
                        await asyncio.sleep(1)
                    else:
                        raise SerialEbbError(e)

        self = cls(reader, writer, command_timeout)

        version = ''.join(await self.run('V')).strip()
        if check_version_response(version):
            logger.info('confirmed EBB (%s) on port %s', version, port)
            await self.run(('CU',1, 1 if self.expect_OK else 0))
        else:
            logger.error('could not confirm EBB on port %s', port)


        return self


    def __init__(self, reader, writer, command_timeout):
        self.reader = reader
        self.writer = writer
        self.command_timeout = command_timeout
        self.expect_OK = True
        self.sent_commands = asyncio.Queue()


    async def run(self, command_or_commands):
        try:
            sent = await self.send(command_or_commands)
            return [response.decode(ENCODING) async for response in self.reiceive()]
        except serial.SerialException as e:
            raise SerialEbbError(e)


    async def send(self, command_or_commands):
        parsed_commands = list(parse_commands(command_or_commands))
        cmds_bytes = serialize_commands(parsed_commands)

        logger.debug('-> %s', repr(cmds_bytes))
        for command in parsed_commands:
            self.sent_commands.put_nowait(command)

        self.writer.write(cmds_bytes)


    async def reiceive(self, total_timeout=0):
        readbuffer = await self.reader.read(512)

        giveup_time = time.time() + total_timeout

        while not self.sent_commands.empty():
            cmd,*args = await self.sent_commands.get()
            cmd = cmd.upper()
            if cmd == 'CU' and args[0] == 1:
                self.expect_OK = bool(args[1])
            
            pattern = RESPONSE_PATTERNS[cmd][0 if self.expect_OK else 1]
            
            giveup_time = max(giveup_time, time.time() + self.command_timeout)

            response_bytes = None
            while time.time() < giveup_time:
                m = pattern.search(readbuffer)
                if m:
                    if m.start()!=0:
                        logger.warn('dropped %s', readbuffer[:m.start()])
                    response_bytes = m.group(0)
                    readbuffer = readbuffer[m.end():]
                    break
                else:
                    readbuffer += await self.reader.read(512)

            if response_bytes != None:
                yield response_bytes
            else:
                logger.warn('no response for %s', format_command((cmd,*args)))

        if readbuffer:
            logging.warn('unexpected response: %s (for %s)', readbuffer, format_commands(parsed_commands))

    def close(self):
        self.writer.close()

    @property
    def port(self):
        return self.writer.transport.serial.port

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def __str__(self):
        return '<AioSerialEbb on %s>' % self.port



def find_all_EBB_ports():
    """Iterate over ports that look like they're an EBB"""
    try:
        import serial.tools.list_ports
        for port,desc,hwid in serial.tools.list_ports.comports():
            if desc.startswith("EiBotBoard") or "VID:PID=04D8:FD92" in hwid.upper():
                yield port
    except ImportError:
        logger.error('could not import `serial.tools.list_ports`')



def find_EBB_port():
    """Find the first port that looks like an EBB"""
    found_ports = list(find_all_EBB_ports())
    if found_ports:
        if len(found_ports) > 1:
            logger.warn('found %d connected EBBs, using first one', len(found_ports))
        return min(found_ports)
    else:
        raise SerialEbbError('could not find any connected EBB')
