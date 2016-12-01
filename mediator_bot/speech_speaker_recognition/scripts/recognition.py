#!/usr/bin/env python
import threading
from collections import deque

import httplib2
import numpy as np
import rospy
import unirest
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
TIMEOUT = None
MAXCONCURRENT = 2

DISCOVERY_URL = ('https://{api}.googleapis.com/$discovery/rest?'
                 'version={apiVersion}')

credentials = None
http = None

def get_speech_service():
    global http, credentials
    credentials = GoogleCredentials.get_application_default().create_scoped(
        ['https://www.googleapis.com/auth/cloud-platform'])
    http = httplib2.Http()
    credentials.authorize(http)

    return discovery.build(
        'speech', 'v1beta1', http=http, discoveryServiceUrl=DISCOVERY_URL)


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


class RequestMethodError(RecogniserNodeException):
    pass


def wrap_http_for_auth(credentials, http):

    orig_request_method = unirest.__request

    def new_request(method, url, params={}, headers={}, auth=None, callback=None):

        # Encode URL
        if params is None:
            params = {}
        url_parts = url.split("\\?")
        url = url_parts[0].replace(" ", "%20")
        if len(url_parts) == 2:
            url += "?" + url_parts[1]

        # Lowercase header keys
        headers = dict((k.lower(), v) for k, v in headers.iteritems())
        headers["user-agent"] = unirest.USER_AGENT

        data, post_headers = unirest.utils.urlencode(params)
        if post_headers is not None:
            headers = dict(headers.items() + post_headers.items())

        headers['Accept-encoding'] = 'gzip'

        if auth is not None:
            if len(auth) == 2:
                user = auth[0]
                password = auth[1]
                encoded_string = base64.b64encode(user + ':' + password)
                headers['Authorization'] = "Basic " + encoded_string

        headers = dict(headers.items() + unirest._defaultheaders.items())


        response, content = http.request(url, method=method, body=data, headers=headers)
        print response
        print content

        # _unirestResponse = unirest.UnirestResponse(response.code,
        #                                            response.headers,
        #                                            response.read())
        #
        # if callback is None or callback == {}:
        #     return _unirestResponse
        # else:
        #     callback(_unirestResponse)

    unirest.__request = new_request



class RPCDispatcher(object):
    # Responsible for queueing up requests and passing the response back correctly
    # Mainly it is there so as to not overflow the Google capacity for our service (I don't want to have pay)

    def __init__(self, period=RPCPERIOD, timeout=TIMEOUT, maxConcurrent=MAXCONCURRENT):
        self.queue = deque()
        self.thread = PeriodicThread(self._sendRequest, period=period)
        self.queueLock = threading.Lock()
        self.timeout = timeout
        self.activeRequests = set()

        self.reqType = {
            'GET': unirest.get,
            'POST': unirest.post,
            'DELETE': unirest.delete,
        }

    def start(self):
        self.thread.start()

    def stop(self):
        self.thread.cancel()

    def queueRequest(self, request, callback):
        with self.queueLock:
            self.queue.appendleft((request, callback))

    def _sendRequest(self):
        with self.queueLock:
            request, callback = self.queue.pop()
        # Google does not support asynchronous requests so transform it into one that does, also fetch token

        self.reqType[request.method.upper()](request.uri, headers=request.headers, params=request.body, callback=self._receiveRequest(callback))
        self.activeRequests.add(request)

    def _receiveRequest(self, callback):
        def requestCallback(response):
            # if request.response is not None:
            #     ret.append(request.response)
            # elif exception_handler and hasattr(request, 'exception'):
            #     ret.append(exception_handler(request, request.exception))
            # else:
            #     ret.append(None)

            # data = response.read().decode('utf8')
            callback(response)

        return requestCallback


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

        service = get_speech_service()
        wrap_http_for_auth(credentials, http)
        service_request = service.speech().syncrecognize(
            body={
                'config': {
                    'encoding': 'LINEAR16',  # raw 16-bit signed LE samples
                    'sampleRate': audioData.sample_rate,  # 16 khz
                    'languageCode': 'en-US',  # a BCP-47 language tag
                },
                'audio': {
                    'content': base64.b64encode(audioData.get_raw_data()).decode('UTF-8')
                }
            })

        self.rpcDispatcher.queueRequest(service_request, lambda x: rospy.loginfo("{}\n{}\n{}".format(x.code, x.headers, x.body)))
        self._transcript = "Test transcript"
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

    def checkTranscriptions(self):
        with self.streamLock:
            for stream in self.srcStreams.itervalues():
                if not stream.hasTranscript() and not stream.active and stream.duration >= MINTRANSCRIBELENGTH:
                    try:
                        transcription = stream.transcript
                        # rospy.loginfo(transcription)
                    except TranscriptionError as e:
                        rospy.logwarn(e)

    def start(self):
        self.rpcDispatcher.start()
        rospy.init_node('SpeakerSpeechNode')
        rospy.on_shutdown(self.rpcDispatcher.stop)
        rospy.Subscriber("HarkSrcWave", HarkSrcWave, self.addToStreams)

        rospy.loginfo("Running")

        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            self.checkTranscriptions()
            rate.sleep()


if __name__ == '__main__':
    node = Node()
    node.start()
