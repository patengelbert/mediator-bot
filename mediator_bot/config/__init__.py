from mediator_bot.common.nested_config_parser import NestedConfigParser


def loadConfig(path='local.conf'):
    conf = NestedConfigParser()
    conf.read(path)
    return conf.load()


