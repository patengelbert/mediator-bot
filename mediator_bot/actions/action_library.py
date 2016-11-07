from mediator_bot.common.bot_module import BotModule


class ActionLibrary(BotModule):

    # (Patrick) I created some sample functions, feel free to remove them
    def getAction(self, *keywords):
        raise NotImplementedError('getAction not yet implemented')

    def getActionChain(self, *keyWords):
        raise NotImplementedError('getActionChain not yet implemented')
