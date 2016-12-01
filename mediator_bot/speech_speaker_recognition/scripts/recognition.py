#!/usr/bin/env python
import threading
from collections import deque
from contextlib import contextmanager

import httplib2
import numpy as np
import rospy
from concurrent.futures import Future
from googleapiclient import discovery
from hark_msgs.msg import HarkSrcWave

from periodic_thread import PeriodicThread
from speech_recognition import Recognizer, AudioData

import base64

from oauth2client.client import GoogleCredentials

# Change this to config file
FRAMELENGTH = 512
SAMPLERATE = 16000
DATATYPE = np.int16
MINTRANSCRIBELENGTH = 1  # Note all transcriptions are billed in intervals of 15s
MAXREQUESTS = 14400 + 50000  # Free + Trial
RPCPERIOD = 1
TIMEOUT = 3
MAXCONCURRENT = 2

DISCOVERY_URL = ('https://{api}.googleapis.com/$discovery/rest?'
                 'version={apiVersion}')


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


class GoogleSocketPool(object):

    def __init__(self, poolSize=MAXCONCURRENT, timeout=TIMEOUT):
        self.sockets = []
        self.inUseSockets = []
        self.credentials = GoogleCredentials.get_application_default().create_scoped(
            ['https://www.googleapis.com/auth/cloud-platform'])

        self.numSockets = threading.Semaphore(poolSize)

        for _ in xrange(poolSize):
            http = httplib2.Http(timeout=timeout, disable_ssl_certificate_validation=True)
            self.credentials.authorize(http)
            self.sockets.append(http)

    @contextmanager
    def getSocket(self):
        rospy.logdebug("Retreiving Socket")
        self.numSockets.acquire()
        socket = self.sockets.pop()
        self.inUseSockets.append(socket)
        yield socket
        rospy.logdebug("Releasing Socket")
        self.inUseSockets.remove(socket)
        self.sockets.append(socket)
        self.numSockets.release()


class RequestMethodError(RecogniserNodeException):
    pass


class RPCDispatcher(object):
    # Responsible for queueing up requests and passing the response back correctly
    # Mainly it is there so as to not overflow the Google capacity for our service (I don't want to have pay)

    def __init__(self, period=RPCPERIOD, timeout=TIMEOUT, maxConcurrentRequests=MAXCONCURRENT):
        self.queue = deque()
        self.thread = PeriodicThread(self._sendRequest, period=period)
        self.queueLock = threading.Lock()
        self.threadLock = threading.Lock()
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
        request = None
        future = None
        rospy.logdebug("Getting queue lock in send request")
        with self.queueLock:
            rospy.logdebug("Got queue lock in send request")
            try:
                request, future = self.queue.pop()
            except IndexError:
                rospy.logdebug("No pending requests")
        if request is None:
            return
        rospy.loginfo("Sending next request")
        # Google does not support asynchronous requests so transform it into one that does, also fetch token
        thread = threading.Thread(target=self.__sendRequest, args=(request, future))
        rospy.logdebug("Getting thread lock on send request")
        with self.threadLock:
            rospy.logdebug("Got thread lock on send request")
            self.activeRequests.append(thread)
            thread.start()

    def __sendRequest(self, body, future):
        thread = threading.currentThread()
        rospy.loginfo("Sending request for thread {}".format(thread.name))
        try:
            with self.createGoogleRequest() as request:
                service = request.speech().syncrecognize(body=body)
                response = service.execute()
        except Exception as e:
            rospy.logerr(e)
            rospy.logdebug("Getting thread lock on send request within thread {}".format(thread.name))
            with self.threadLock:
                rospy.logdebug("Got thread lock on send request within thread {}".format(thread.name))
                self.activeRequests.remove(thread)
            return
        rospy.loginfo("Received response {} for thread {}".format(response, thread.name))
        future.set_result(response)
        rospy.logdebug("Getting thread lock on send request within thread {}".format(thread.name))
        with self.threadLock:
            rospy.logdebug("Got thread lock on send request within thread {}".format(thread.name))
            self.activeRequests.remove(thread)


class SrcStream(object):
    def __init__(self, srcId, sampleRate, rpcDispatcher):
        self.rpcDispatcher = rpcDispatcher
        self.sampleRate = sampleRate
        self._speaker = None
        self.srcId = srcId
        self.data = []
        self.seqIds = []  # List of seqId, startSample, endSample
        self.active = True
        self._transcript = None

    def addFrame(self, seqId, src):
        if src.id != self.srcId:
            raise InvalidSrcException(
                "Cannot attach frame from source stream {} to frames of source stream {}".format(src.id, self.srcId))
        if not self.active:
            raise SrcStreamEndedException("Cannot attach frame to ended source stream {}".format(self.srcId))

        if src is None:
            self.endStream()
        else:
            self.seqIds.append((seqId, self.length, self.length + src.length - 1))
            self.data += src.wavedata

    def endStream(self):
        if not self.active:
            raise SrcStreamEndedException("Stream {} has already been ended".format(self.srcId))
        self.active = False
        # End of data for this stream
        rospy.loginfo("End of data for stream {} after {}s".format(self.srcId, self.duration))
        if self.duration > MINTRANSCRIBELENGTH:
            # TODO send message if speaker also recognised
            rospy.loginfo(self.transcript)

    @property
    def speaker(self):
        if self._speaker is not None:
            return self._speaker

    def hasTranscript(self):
        return self._transcript is not None

    @property
    def transcript(self):
        if self._transcript is not None:
            return self._transcript

        if self.active:
            raise SrcStreamStillActiveException(
                "Stream {} cannot be transcribed while it is still active".format(self.srcId))
        if self.duration < MINTRANSCRIBELENGTH:
            raise TranscriptionError(
                "Stream {} is too short to transcribe {} < {}".format(self.srcId, self.duration, MINTRANSCRIBELENGTH))

        rospy.loginfo("Beginning transcription for stream {}".format(self.srcId))

        audioData = AudioData(np.array(self.data, np.int16).tostring(), SAMPLERATE, 2)

        transcript = self.rpcDispatcher.queueRequest({
                'config': {
                    'encoding': 'LINEAR16',  # raw 16-bit signed LE samples
                    'sampleRate': audioData.sample_rate,  # 16 khz
                    'languageCode': 'en-US',  # a BCP-47 language tag
                },
                'audio': {
                    'content': base64.b64encode(audioData.get_raw_data()).decode('UTF-8')
                }
            })
        # TODO convert transcript JSON into usable data
        self._transcript = transcript.result()
        return self._transcript

    def __len__(self):
        return self.length

    @property
    def length(self):
        return len(self.data)

    @property
    def duration(self):
        return self.length / float(self.sampleRate)


class Node(object):
    def __init__(self):
        self.srcStreams = {}
        self.activeStreams = set()
        self.streamLock = threading.Lock()

        self.rpcDispatcher = RPCDispatcher()

    def addToStreams(self, data):
        with self.streamLock:
            if data.exist_src_num == 0:
                for streamId in self.activeStreams:
                    self.srcStreams[streamId].endStream()
                self.activeStreams.clear()
            else:
                updatedStreams = set()
                for srcStream in data.src:
                    streamId = srcStream.id
                    if streamId not in self.srcStreams.keys():
                        self.srcStreams[streamId] = SrcStream(streamId, SAMPLERATE, self.rpcDispatcher)
                        self.activeStreams.add(streamId)
                    self.srcStreams[streamId].addFrame(data.header.seq, srcStream)
                    updatedStreams.add(streamId)

                for streamId in [sid for sid in self.activeStreams if sid not in updatedStreams]:
                    self.activeStreams.remove(streamId)
                    self.srcStreams[streamId].endStream()

    def start(self):
        self.rpcDispatcher.start()
        rospy.init_node('SpeakerSpeechNode')
        rospy.on_shutdown(self.rpcDispatcher.stop)
        rospy.Subscriber("HarkSrcWave", HarkSrcWave, self.addToStreams)

        rospy.loginfo("Running")
        rospy.spin()

if __name__ == '__main__':
    node = Node()
    node.start()
