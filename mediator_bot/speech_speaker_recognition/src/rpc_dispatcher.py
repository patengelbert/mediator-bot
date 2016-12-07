import threading
from collections import deque
from contextlib import contextmanager

import rospy
from concurrent.futures import Future
from googleapiclient import discovery

from config import RPCPERIOD, TIMEOUT, MAXCONCURRENT, DISCOVERY_URL
from locks import TimeoutLock
from periodic_thread import PeriodicThread
from socket_pool import GoogleSocketPool


class RPCDispatcher(object):
    # Responsible for queueing up requests and passing the response back correctly
    # Mainly it is there so as to not overflow the Google capacity for our service (I don't want to have pay)

    def __init__(self, period=RPCPERIOD, timeout=TIMEOUT, maxConcurrentRequests=MAXCONCURRENT):
        self.queue = deque()
        self.thread = PeriodicThread(self._sendRequest, period=period)
        self.queueLock = TimeoutLock()
        self.threadLock = TimeoutLock()
        self.activeRequests = []

        self.socketPool = GoogleSocketPool(maxConcurrentRequests, timeout)

    @contextmanager
    def createGoogleRequest(self):
        with self.socketPool.getSocket() as http:
            yield discovery.build('speech', 'v1beta1', http=http, discoveryServiceUrl=DISCOVERY_URL)

    def start(self):
        self.thread.start()

    def stop(self):
        self.thread.cancel()
        with self.threadLock:
            activeRequests = self.activeRequests
        for request in activeRequests:
            request.join()

    def queueRequest(self, request):
        future = Future()
        rospy.logdebug("Getting queue lock in queue request")
        with self.queueLock:
            rospy.logdebug("Got queue lock in queue request")
            self.queue.appendleft((request, future))
        return future

    def _sendRequest(self):
        rospy.logdebug("Getting queue lock in send request")
        with self.queueLock:
            rospy.logdebug("Got queue lock in send request")
            try:
                request, future = self.queue.pop()
            except IndexError:
                rospy.logdebug("No pending requests")
                return
        rospy.logdebug("Sending next request")
        # Google does not support asynchronous requests so transform it into one that does, also fetch token
        thread = threading.Thread(target=self.__sendRequest, args=(request, future))
        rospy.logdebug("Getting thread lock on send request")
        with self.threadLock:
            rospy.logdebug("Got thread lock on send request")
            self.activeRequests.append(thread)
            thread.start()

    def __sendRequest(self, body, future):
        thread = threading.currentThread()
        try:
            with self.createGoogleRequest() as request:
                service = request.speech().syncrecognize(body=body)
                rospy.logdebug("Sending request for thread {}".format(thread.name))
                response = service.execute()
        except Exception as e:
            rospy.logdebug("Getting thread lock on send request within thread {}".format(thread.name))
            with self.threadLock:
                rospy.logdebug("Got thread lock on send request within thread {}".format(thread.name))
                self.activeRequests.remove(thread)
            future.set_exception(e)
            return
        rospy.logdebug("Received response {} for thread {}".format(response, thread.name))
        future.set_result(response)
        rospy.logdebug("Getting thread lock on send request within thread {}".format(thread.name))
        with self.threadLock:
            rospy.logdebug("Got thread lock on send request within thread {}".format(thread.name))
            self.activeRequests.remove(thread)
