"""
  Applications launcher

example:
    launch(app=GoOneMeter, description='Go One Meter', prefix='go1m-')
"""

from osgar.lib.config import load as config_load
from osgar.record import Recorder
from osgar.replay import replay

from osgar.logger import LogWriter
import argparse


def launch(app, description, prefix):
    parser = argparse.ArgumentParser(description=description)
    subparsers = parser.add_subparsers(help='sub-command help', dest='command')
    subparsers.required = True
    parser_run = subparsers.add_parser('run', help='run on real HW')
    parser_run.add_argument('config', nargs='+', help='configuration file')
    parser_run.add_argument('--note', help='add description')

    parser_replay = subparsers.add_parser('replay', help='replay from logfile')
    parser_replay.add_argument('logfile', help='recorded log file')
    parser_replay.add_argument('--force', '-F', dest='force', action='store_true', help='force replay even for failing output asserts')
    parser_replay.add_argument('--config', nargs='+', help='force alternative configuration file')
    parser_replay.add_argument('--verbose', '-v', help="verbose mode", action='store_true')
    args = parser.parse_args()

    if args.command == 'replay':
        args.module = 'app'
        game = replay(args, application=app)
        game.verbose = args.verbose
        game.run()

    elif args.command == 'run':
        log = LogWriter(prefix=prefix, note=str(sys.argv))
        config = config_load(*args.config)
        log.write(0, bytes(str(config), 'ascii'))  # write configuration
        robot = Recorder(config=config['robot'], logger=log, application=app)
        game = robot.modules['app']  # TODO nicer reference
        robot.start()
        game.run()
        robot.finish()

# vim: expandtab sw=4 ts=4
