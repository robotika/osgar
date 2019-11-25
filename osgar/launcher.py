"""
  Applications launcher

example:
    launch(app=GoOneMeter, description='Go One Meter', prefix='go1m-')
"""
import argparse

from osgar.record import record
from osgar.replay import replay
from osgar.lib.config import config_load


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
        cfg = config_load(*args.config, application=args.application)
        record(cfg, log_prefix=prefix)

# vim: expandtab sw=4 ts=4
