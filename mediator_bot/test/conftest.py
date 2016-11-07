import pytest
from DictObject import DictObject

from mediator_bot.actions.action_library import ActionLibrary
from mediator_bot.bot import MediatorBot


@pytest.fixture
def defaultConfig():
    return DictObject(
        GENERAL=DictObject(
            debug=True,
        ),
        ROBOT=DictObject(
            port="3000",
            host="0.0.0.0",
        )
    )


@pytest.fixture
def mediatorBot(defaultConfig):
    return MediatorBot(config=defaultConfig)


@pytest.fixture
def actionLibrary(defaultConfig):
    return ActionLibrary(defaultConfig)
