import pytest
from DictObject import DictObject

from mediator_bot.common.nested_config_parser import NestedConfigParser


@pytest.fixture
def parser():
    config = NestedConfigParser()
    config.add_section('Section1')
    config.set('Section1', 'testvar', 15)

    config.add_section('Section2')
    config.set('Section2', 'testvar', 16)

    return config


class TestNestedConfigParser:
    def test_load_config(self, parser):
        config = parser.load()

        assert type(config) == DictObject
        assert config.Section1.testvar == 15
        assert config.Section2.testvar == 16