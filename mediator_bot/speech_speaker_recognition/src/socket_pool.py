from contextlib import contextmanager

import httplib2
import rospy
from oauth2client.client import GoogleCredentials

from config import MAXCONCURRENT, TIMEOUT
from custom_exceptions import LockTimeoutException
from locks import TimeoutSemaphore


class GoogleSocketPool(object):
    def __init__(self, poolSize=MAXCONCURRENT, timeout=TIMEOUT):
        self.sockets = []
        self.inUseSockets = []
        self.credentials = GoogleCredentials.get_application_default().create_scoped(
            ['https://www.googleapis.com/auth/cloud-platform'])

        self.numSockets = TimeoutSemaphore(poolSize)

        for _ in xrange(poolSize):
            http = httplib2.Http(timeout=timeout, disable_ssl_certificate_validation=True)
            self.credentials.authorize(http)
            self.sockets.append(http)

    @contextmanager
    def getSocket(self):
        rospy.logdebug("Retrieving Socket")

        try:
            self.numSockets.acquire()
        except LockTimeoutException as e:
            rospy.logerr(e)
            raise

        socket = self.sockets.pop()
        self.inUseSockets.append(socket)
        yield socket
        rospy.logdebug("Releasing Socket")
        self.inUseSockets.remove(socket)
        self.sockets.append(socket)
        self.numSockets.release()
