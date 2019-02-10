from collections import namedtuple
import serial
import time
import threading
import logging


logger = logging.getLogger(__name__)


class SerialEbbError(IOError):
    pass


class Ebb():
    COMMANDS = [
        'A','AC','BL','C','CN','CK','CU','CS','EM','ES','I','LM','MR','MW','ND','NI',
        'O', 'PC','PD','PG','PI','PO','QB','QC','QL','QM','QN','QP','QR','QS','QT',
        'RB','R','S2','SC','SE','SL','SM','SN','SP','SR','ST','T','TP','V','XM'
    ]
    def __init__(self):
        def proxy(c):
            def f(*args):
                return self.run(c, *args)
            return f

        for cmd in self.COMMANDS:
            if not hasattr(self, cmd):
                setattr(self, cmd, proxy(cmd))



class SerialEbb(Ebb):
    ENCODING = 'ascii'
    READ_TIMEOUT = 0.025
    TOTAL_TMEOUT = 2


    def __init__(self, port=None, *args, **kwargs):
        super().__init__(*args, **kwargs)

        if not port:
            found_ports = list(find_EBB_ports())
            if found_ports:
                if len(found_ports) > 1:
                    logger.warn('found %d connected EBBs, using first one', len(found_ports))
                port = min(found_ports)
            else:
                raise SerialEbbError('could not find any connected EBB')

        self.open(port)


    def open(self, port):
        try:
            logger.debug('opening serial port `%s`', port)
            self.serialport = serial.Serial(port, timeout=self.READ_TIMEOUT)
            ebb_info = check_serialport(self.serialport)
            if not ebb_info:
                self.serialport.close()
                logger.warn('EBB check failed, closing serial port')
                raise SerialEbbError('could not confirm EBB version on port %s' % port)
            else:
                logger.info('confirmed EBB board %s on port `%s`', ebb_info, port)
                self.serialport.reset_input_buffer()
        except OSError as e:
            if e.errno == 16:
                raise SerialEbbError('port `%s` busy' % port)
            else:
                raise SerialEbbError(e.strerror)


    def reopen(self):
        self.close()
        self.open(self.serialport.port)


    def run(self, *command):
        cmd_bytes = self.parse_command(command)

        logger.debug('-> %s', repr(cmd_bytes))
        t0 = time.time()

        self.serialport.reset_input_buffer()
        self.serialport.write(cmd_bytes)
        response = self.read_response(cmd_bytes)
        
        t1 = time.time()
        logger.debug('<- %s (%fs)', repr(response), t1-t0)

        return response


    def read_response(self, cmd):

        def nlcr_terminated(r, endswith=b''):
            # it looks like some responses end in `\n\r` instead of `\r\n`
            # (eg. `QM` in version `2.5.1`)
            return r.endswith(endswith + b'\r\n') \
                or r.endswith(endswith + b'\n\r')

        # preicate matching a complete successful response
        def ok(r):
            #FIXME handle other commands that don't end response with `OK<NL><CR>`
            if cmd.lower() in [b'v\r', b'qm\r'] :
                return nlcr_terminated(r)
            else:
                return nlcr_terminated(r, endswith=b'OK')

        # preicate matching a complete unsuccessful response
        def error(r):
            return r.startswith(b'!') and nlcr_terminated(r)


        # accumulate fragments until the whole string looks like a complete response
        response = b''
        giveup_time = time.time() + self.TOTAL_TMEOUT
        while time.time() < giveup_time:
            response += self.serialport.readline()
            if ok(response):
                return bytearray(response)
            if error(response):
                raise SerialEbbError('%s %s' % (cmd, self.decode(response).strip()))

        # we didn't exit early, some termination cases must be missing :/
        logger.warn('unexpected response: %s %s', cmd, response)
        return bytearray(response)


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
        return '<EBB on %s>' % self.port


    @classmethod
    def parse_command(cls, cmd):
        if isinstance(cmd, (bytes, bytearray)):
            # cmd is already bytes: just pass it along
            return cmd
        if isinstance(cmd, str):
            # cmd is a string: ensure it is '\r'-terminated and encode to bytes
            return bytearray(cmd if cmd.endswith('\r') else cmd+'\r', cls.ENCODING)
        if isinstance(cmd, tuple):
            # cmd is a tuple: convert each element, join with comma, encode to bytes
            return bytearray(','.join(cls.parse_tuple_command_iter(*cmd))+'\r', cls.ENCODING)

    @classmethod
    def parse_tuple_command_iter(cls, *values):
        for value in values:
            if isinstance(value, float):
                yield str(int(round(value)))
            else:
                yield str(value)


    @classmethod
    def decode(cls, bytes_str):
        """decode response bytes to a string"""
        return bytes_str.decode(cls.ENCODING)



def find_EBBs():
    for port in find_EBB_ports():
        try:
            with serial.Serial(port, timeout=SerialEbb.READ_TIMEOUT) as serialport:
                ebb_info = check_serialport(serialport)
                if ebb_info:
                    yield port, ebb_info
        except (OSError, serial.SerialException) as e:
            pass


def find_EBB_ports():
    """Iterate over ports that look like they're an EBB"""
    try:
        import serial.tools.list_ports
        for port, desc, hwid in serial.tools.list_ports.comports():
            if desc.startswith("EiBotBoard") or "VID:PID=04D8:FD92" in hwid.upper():
                yield port
    except ImportError:
        logger.error('could not import `serial.tools.list_ports`')



EBBInfo = namedtuple('EBBInfo', 'hardware firmware nickname')


def check_serialport(serialport, retries=4):
    """Check if an open serial port is an actual EBB by queriying its version and nickname"""
    for i in range(retries):
        serialport.write(b'v\r')
        response = serialport.readline()
        if response and response.startswith(b'EBB'):
            split = response.strip().split()
            hardware, firmware = split[0], split[-1]

            serialport.write(b'qt\r')
            response2 = serialport.readline() + serialport.readline() #FIXME
            if response2.endswith(b'\r\nOK\r\n'):
                nickname = response2[:-6]
            else:
                nickname = None

            return EBBInfo(hardware, firmware, nickname)
        else:
            logger.debug('failed to read EBB version, retrying...')

