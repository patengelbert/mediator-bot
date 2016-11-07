

class BotModule(object):
    def __init__(self, config):
        self.config = config

    def execute(self, *args, **kwargs):
        raise NotImplementedError