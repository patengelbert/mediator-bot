import ConfigParser

import pytest

import mediator_bot
from mediator_bot.bot import MediatorBot


@pytest.fixture
def configFile(tmpdir):
    conf = tmpdir.join("testConfig.conf")

    config = ConfigParser.ConfigParser()

    config.add_section('GENERAL')
    config.set('GENERAL', 'debug', 'False')
    config.add_section('ROBOT')
    config.set('ROBOT', 'port', '5000')
    config.set('ROBOT', 'host', '0.0.0.0')

    # It seems that tmpdir always overwrote itself
    with open(str(conf), "w+") as f:
        config.write(f)

    return conf


class TestMediatorBot:
    def test_version(self):
        assert mediator_bot.__version__ >= "1.0.0"

    def test_config(self, mediatorBot):
        assert mediatorBot.config == {
            'GENERAL': {
                'debug': True,
            },
            'ROBOT': {
                'port': '3000',
                'host': '0.0.0.0',
            },
            'ACTIONS': {},
            'AI': {},
            'NLP': {},
            'PREPROCESSING': {},
            'SPEECHCONVERSION': {},
            'SPEECHDIARIZATION': {},
            'UI': {}
        }

    def test_loads_file_correct(self, configFile):
        bot = MediatorBot(configPath=str(configFile))
        assert bot.config == {
            'GENERAL': {
                'debug': False,
            },
            'ROBOT': {
                'port': '5000',
                'host': '0.0.0.0',
            },
            'ACTIONS': {},
            'AI': {},
            'NLP': {},
            'PREPROCESSING': {},
            'SPEECHCONVERSION': {},
            'SPEECHDIARIZATION': {},
            'UI': {}
        }
