import ConfigParser
from DictObject import DictObject


class NestedConfigParser(ConfigParser.ConfigParser):

    def load(self):
        d = DictObject(self._sections)
        for k in d:
            d[k] = DictObject(self._defaults, DictObject(**d[k]))
            d[k].pop('__name__', None)
        return d