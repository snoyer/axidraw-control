import os
import logging
import argparse
import json


from axidrawcontrol import SerialEbb
from axidrawcontrol.axidraw import CommandsBuilder, parse_settings
from axidrawcontrol.plotter import do_plot
from axidrawcontrol.util.misc import pairwise
from axidrawcontrol.util.svg import find_paths

def main():
    argparser = argparse.ArgumentParser()
    argparser.add_argument('svg')
    argparser.add_argument('--port')
    argparser.add_argument('--settings')
    argparser.add_argument('--scale', type=float, default=1)


    args = argparser.parse_args()

    logging.basicConfig(level=logging.DEBUG)

    logging.getLogger('axidrawcontrol.ebb').setLevel(logging.WARN)
    logging.getLogger('axidrawcontrol.motionplanning').setLevel(logging.WARN)


    settings = parse_settings()

    if args.settings:
        fn = args.settings
        if os.path.isfile(fn):
            try:
                settings = parse_settings(json.load(open(fn)))
            except json.JSONDecodeError:
                logger.error('cannot parse `%s` as json', fn)
        else:
            logger.error('cannot read `%s`', fn)

    print(settings)


    def ebb_commands():
        cb = CommandsBuilder()
        for path in find_paths(args.svg, scale=args.scale):
            yield from cb.draw_path(path, settings)
        yield from cb.walk_path([0j], settings)


    with SerialEbb(args.port) as ebb:
        try:
            do_plot(ebb, list(ebb_commands()))
            # do_plot(ebb, ebb_commands())
        except KeyboardInterrupt:
            pass
        finally:
            ebb.run(('EM',0,0))



if __name__ == '__main__':
    main()
