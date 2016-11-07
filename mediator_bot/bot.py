import argparse
import os
import sys

sys.path += [os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))]

from mediator_bot import config as configModule
from mediator_bot.actions.actionLibrary import ActionLibrary


class MediatorBot(object):

    config = {}

    def __init__(self, config=None, bind=None, port=None, debug=False):
        # Setup config

        if config is None:
            config = os.path.abspath(os.path.join(os.path.dirname(__file__), 'config', 'local.conf'))

        configModule.loadConfig(self, config)
        self.config['debug'] = debug

        if bind is not None:
            self.config['bind'] = bind
        if port is not None:
            self.config['port'] = port

        self.actionLibrary = ActionLibrary()

    def run(self):
        pass


def parseArguments(args=None):
    if args is None:
        args = sys.argv[1:]

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', '-p', type=int, default="3000", help="Port at which to find the robot.")
    parser.add_argument('--bind', '-b', default="0.0.0.0", help="IP at which to find the robot.")
    parser.add_argument('--conf', '-c', default='mediator_bot/config/local.conf', help="Path to configuration file.")
    parser.add_argument('--debug', '-d', action='store_true', default=False,
                        help='Indicates whether to run in debug mode.')

    options = parser.parse_args(args)

    return options


if __name__ == '__main__':
    args = parseArguments()
    bot = MediatorBot(args.conf, args.bind, args.port, args.debug)
    bot.run()
