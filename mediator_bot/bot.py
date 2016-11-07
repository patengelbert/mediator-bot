import argparse
import os
import sys

from mediator_bot import config as configModule
from mediator_bot.actions.action_library import ActionLibrary
from mediator_bot.ai.ai_module import AIModule

from DictObject import DictObject

sys.path += [os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))]


class MediatorBot(object):
    def __init__(self, configPath=None, host=None, port=None, debug=False, config=None):

        # Define in here to not have static variable
        self.config = DictObject(
            GENERAL={},
            ROBOT={},
            PREPROCESSING={},
            NLP={},
            AI={},
            ACTIONS={},
            SPEECHCONVERSION={},
            SPEECHDIARIZATION={},
            UI={},
        )

        # Setup config
        if configPath is None and config is None:
            configPath = os.path.abspath(os.path.join(os.path.dirname(__file__), 'config', 'local.conf'))

        if config is None:
            c = configModule.loadConfig(configPath)

            self.config.update(c)

            self.config.GENERAL.debug = debug

            if host is not None:
                self.config.ROBOT.host = host
            if port is not None:
                self.config.ROBOT.port = port
        else:
            self.config.update(config)

        # Add new modules here
        self.aiModule = AIModule(self.config)
        self.actionLibrary = ActionLibrary(self.config)

    def run(self):
        pass


def parseArguments(args=None):
    if args is None:
        args = sys.argv[1:]

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', '-p', type=int, help="Port at which to find the robot.")
    parser.add_argument('--host', '-h', help="IP at which to find the robot.")
    parser.add_argument('--conf', '-c', help="Path to configuration file.")
    parser.add_argument('--debug', '-d', action='store_true', help='Indicates whether to run in debug mode.')

    options = parser.parse_args(args)

    return options


if __name__ == '__main__':
    args = parseArguments()
    bot = MediatorBot(args.conf, args.host, args.port, args.debug)
    bot.run()
