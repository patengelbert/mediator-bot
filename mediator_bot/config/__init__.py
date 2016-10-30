from derpconf.config import Config, generate_config


def loadConfig(bot, path=None):
    conf = Config.load(path)
    for conf_option, _ in conf.items.items():
        bot.config[conf_option] = conf[conf_option]


if __name__ == '__main__':
    generate_config()
