import serial
import logging


logger = logging.getLogger(__name__)


class EBBSerialError(IOError):
    pass


class EBB(object):
    ENCODING = 'ascii'


    def __init__(self, port=None, timeout=1.0):
        if not port:
            found_ports = list(self.find_ports())
            if found_ports:
                port = found_ports[0]
                if len(found_ports) > 1:
                    logger.warn('found %d connected EBBs, using first one', len(found_ports))
            else:
                raise EBBSerialError('could not find any connected EBB')

        try:
            logger.info('opening serial port')
            serialport = serial.Serial(port, timeout=timeout)
            if not self.check_serialport(serialport):
                serialport.close()
                logger.warn('EBB check failed, closing serial port')
                raise EBBSerialError('could not confirm EBB version on %s' % port)

            self._serialport = serialport
        except OSError as e:
            if e.errno == 16:
                raise EBBSerialError('EBB busy')
            else:
                raise EBBSerialError(e.strerror)


    def run(self, *commands):
        return b''.join(self.run_commands_iter(commands))


    def run_commands_iter(self, commands):
        return list(self.run_commands(commands))


    def run_commands_iter(self, commands):
        for command in commands:
            cmd_bytes = self.parse_command(command)

            logger.debug('-> %s', cmd_bytes)
            self._serialport.write(cmd_bytes)
            response = self.read_response(cmd_bytes)
            logger.debug('<- %s', response)
            yield response


    def read_response(self, cmd):

        def nlcr_terminated(r, endswith=b''):
            # it looks like some responses end in `\n\r` instead of `\r\n`
            # (eg. `QM` in version `2.5.1`)
            return r.endswith(endswith + b'\r\n') \
                or r.endswith(endswith + b'\n\r')

        # stop predicate to know when to stop reading the output stream
        def ok(r):
            #FIXME handle other commands that don't end response with `OK<NL><CR>`
            if cmd.lower() in [b'v\r', b'qm\r'] :
                return nlcr_terminated(r)
            return nlcr_terminated(r, endswith=b'OK')

        # stop predicate to know when to stop reading the output stream
        def error(r):
            return r.startswith(b'!') and nlcr_terminated(r)


        # accumulate fragments until the whole string looks like a complete response
        response = b''
        for i in range(32): # arbitrary number of retries so we never hang forever
            response += self._serialport.readline()
            if ok(response):
                return response
            if error(response):
                raise EBBSerialError('%s %s' % (cmd, EBB.decode(response).strip()))

        # we didn't exit early, some termination cases must be missing :/
        logger.warn('unexpected response: %s %s', cmd, response)
        return response



    def close(self):
        if self._serialport:
            logger.info('closing serial port')
            self._serialport.close()
        else:
            logger.warn('no serial port to close')

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()


    def port(self):
        return self._serialport.port


    def __str__(self):
        return '<EBB on %s>' % self.port()


    @staticmethod
    def parse_command(cmd):
        if isinstance(cmd, bytes):
            # cmd is already bytes: just pass it along
            return cmd
        if isinstance(cmd, str):
            # cmd is a string: ensure it is '\r'-terminated and encode to bytes
            return bytes(cmd if cmd.endswith('\r') else cmd+'\r', EBB.ENCODING)
        if isinstance(cmd, tuple):
            # cmd is a tuple: convert each element, join with comma, encode to bytes
            return bytes(','.join(EBB.parse_tuple_command_iter(*cmd))+'\r', EBB.ENCODING)

    @staticmethod
    def parse_tuple_command_iter(*values):
        for value in values:
            if isinstance(value, float):
                yield str(int(round(value)))
            else:
                yield str(value)


    @staticmethod
    def decode(bytes_str):
        """decode response bytes to a string"""
        return str(bytes_str, EBB.ENCODING)


    @staticmethod
    def find_ports():
        """Iterate over ports that look like they're an EBB"""
        try:
            import serial.tools.list_ports
            for port, desc, hwid in serial.tools.list_ports.comports():
                if desc.startswith("EiBotBoard") \
                or hwid.startswith("USB VID:PID=04D8:FD92"):
                    yield port
        except ImportError:
            logger.error('could not import `serial.tools.list_ports`')


    @staticmethod
    def check_serialport(serialport, retries=4):
        """Check if an open serial port is an actual EBB by queriying its version"""
        for i in range(retries):
            serialport.write(b'v\r')
            response = serialport.readline()
            if response and response.startswith(b'EBB'):
                return True
            logger.debug('failed to read EBB version, retrying')
        return False
