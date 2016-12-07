
class RecogniserNodeException(Exception):
    def __init__(self, what="Exception in Recogniser Node"):
        self.what = what

    def __str__(self):
        return self.what


class InvalidSrcException(RecogniserNodeException):
    pass


class SrcStreamEndedException(RecogniserNodeException):
    pass


class SrcStreamStillActiveException(RecogniserNodeException):
    pass


class TranscriptionError(RecogniserNodeException):
    pass


class RequestError(RecogniserNodeException):
    pass


class SpeakerRecognitionError(RecogniserNodeException):
    pass


class WouldBlockError(Exception):
    pass


class LockTimeoutException(Exception):
    pass


class RequestMethodError(RecogniserNodeException):
    pass


class UnknownObjectError(Exception):
    pass

