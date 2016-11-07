from mediator_bot.config.nested_config_parser import NestedConfigParser


def loadConfig(bot, path='local.conf'):
    conf = NestedConfigParser()
    conf.read(path)
    bot.config = conf.load()


