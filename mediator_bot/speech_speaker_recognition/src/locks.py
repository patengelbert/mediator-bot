from Queue import Queue, Empty
from contextlib import contextmanager

from custom_exceptions import LockTimeoutException, WouldBlockError

from config import TIMEOUT


class TimeoutSemaphore(object):
    def __init__(self, size=1, timeout=TIMEOUT):
        self.timeout = timeout
        self.queue = Queue(maxsize=size)
        for _ in xrange(size):
            self.release()

    def acquire(self, blocking=True):
        try:
            self.queue.get(block=blocking, timeout=self.timeout)
        except Empty as e:
            raise LockTimeoutException(str(e))

    def release(self):
        self.queue.put(1, block=False)


class TimeoutLock(TimeoutSemaphore):
    def __init__(self, timeout=TIMEOUT):
        super(TimeoutLock, self).__init__(size=1, timeout=timeout)

    def __enter__(self):
        self.acquire()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()


@contextmanager
def non_blocking_lock(lock=TimeoutLock()):
    if not lock.acquire(blocking=False):
        raise WouldBlockError()
    try:
        yield lock
    finally:
        lock.release()