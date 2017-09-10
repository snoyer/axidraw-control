import logging

from axidrawcontrol import EBB, EBBSerialError

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    try:
        with EBB() as ebb:
            print('EBB version: %s', EBB.decode(ebb.run('V\r')).strip())
    except EBBSerialError as e:
        print('error:', e)
