#!/usr/bin/env python
import threading

import datetime
import rospy
import std_msgs.msg
from enum import Enum
from hark_msgs.msg import HarkSrcWave
from speaker_recognition import SpeakerRecognizer
from speech_speaker_recognition.msg import SentenceTranscription, Speaker, AddedUser, StartRecognitionMsg
from speech_speaker_recognition.srv import StartEnrollment, EndEnrollment, StartRecognition, SaveModel, LoadModel
from config import (
    SPEAKERMODEL,
    MINTRANSCRIBELENGTH,
    MINDIARIZELENGTH,
    SAMPLERATE,
    SAVELOADEVENTSUPPORTED,
)
from rpc_dispatcher import RPCDispatcher
from src_stream import SrcStream, StreamType


class Actions(Enum):
    Unknown = 0
    Recognise = 1
    Enroll = 2
    Starting = 3
    Stopping = 4


def addHeader(msg):
    msg.header = std_msgs.msg.Header()
    msg.header.stamp = rospy.Time.now()


class Node(object):
    def __init__(self, modelFile=None, debugLevel=rospy.INFO):
        rospy.init_node('SpeakerSpeechNode', log_level=debugLevel)

        rospy.loginfo("Initialising")

        self._action = Actions.Starting
        self.srcStreams = dict()
        self.activeStreams = set()
        self.streamLock = threading.Lock()  # Used to prevent all streams from being modified while processing is occurring
        self.actionLock = threading.Lock()

        self.rpcDispatcher = RPCDispatcher()
        self.recogniser = SpeakerRecognizer()
        self.transcribingStreams = []

        rospy.on_shutdown(self.rpcDispatcher.stop)

        rospy.logdebug("Creating Publishers/Subscribers")

        self.transcriptionPublisher = rospy.Publisher('transcriptions', SentenceTranscription, queue_size=10)
        self.speakerPublisher = rospy.Publisher('speaker', Speaker, queue_size=10)
        self.userAddedPublisher = rospy.Publisher("added_user", AddedUser, queue_size=4, latch=True)
        self.startedPublisher = rospy.Publisher("start_recognition", StartRecognitionMsg, queue_size=1, latch=True)

        rospy.Subscriber("HarkSrcWave", HarkSrcWave, self.addToStreams)

        rospy.Service('StartEnrollment', StartEnrollment, self.enroll)
        rospy.Service('EndEnrollment', EndEnrollment, self.endEnroll)
        rospy.Service('StartRecognition', StartRecognition, self.startRecognition)
        rospy.Service('SaveModel', SaveModel, self.saveModel)
        rospy.Service('LoadModel', LoadModel, self.loadModel)

        self.enrollName = None
        self._savedAction = None
        self.numAdded = 0
        self.trained = True

        rospy.logdebug("Waiting for subscribers to be set up")
        rospy.sleep(3)

        # TODO it send messages to soon here
        if modelFile is not None:
            # Wat for subscribers to have set up
            rospy.logdebug("Loading model file")
            self._loadModel(modelFile)

        if len(self.recogniser.features) == 0:
            rospy.logwarn("Please enroll people before starting detection")

        rospy.logdebug("Finished initialisation")

    @property
    def action(self):
        with self.actionLock:
            return self._action

    @action.setter
    def action(self, val):
        with self.actionLock:
            rospy.loginfo("Changing mode to {}".format(val.name))
            self._action = val

    def addToStreams(self, data):

        if self.action not in (Actions.Recognise, Actions.Enroll):
            return

        with self.streamLock:
            if data.exist_src_num == 0:
                for streamId in self.activeStreams:
                    self.srcStreams[streamId].endStream(data.header.stamp)
                self.activeStreams.clear()
            else:
                updatedStreams = set()
                for srcStream in data.src:
                    streamId = srcStream.id
                    if streamId not in self.srcStreams.keys():
                        streamType = StreamType.ENROLLMENT if self.action == Actions.Enroll else StreamType.RECOGNITION
                        self.srcStreams[streamId] = SrcStream(streamId, SAMPLERATE, self.rpcDispatcher, self.recogniser,
                                                              rospy.Time.now(), streamType=streamType,
                                                              speaker=self.enrollName if streamType == StreamType.ENROLLMENT else None)
                        self.activeStreams.add(streamId)

                    self.srcStreams[streamId].addFrame(data.header.seq, rospy.Time.now(), srcStream)
                    updatedStreams.add(streamId)

                for streamId in [sid for sid in self.activeStreams if sid not in updatedStreams]:
                    self.activeStreams.remove(streamId)
                    self.srcStreams[streamId].endStream(data.header.stamp)

    def checkEnrollStream(self):
        with self.streamLock:
            streamsToCheck = [stream for stream in self.srcStreams.itervalues() if stream.type == StreamType.ENROLLMENT]

            for stream in streamsToCheck:
                if not stream.active and not stream.enrolled and stream.duration > MINDIARIZELENGTH:
                    stream.enroll()

    def checkStreams(self):
        with self.streamLock:
            streamsToCheck = [stream for stream in self.srcStreams.itervalues() if
                              stream.type == StreamType.RECOGNITION and
                              (not stream.hasKnownSpeaker() or not stream.hasTranscription())]
        for stream in streamsToCheck:
            doTranscription = not stream.active and stream.duration > MINTRANSCRIBELENGTH and not stream.hasTranscription()
            doSpeakerRecognition = stream.duration > MINDIARIZELENGTH
            if doTranscription:
                if stream.srcId in self.transcribingStreams:
                    transcript = stream.checkTranscription()
                    if transcript is not None:
                        self.transcribingStreams.remove(stream.srcId)
                else:
                    stream.doTranscription(async=True)
                    self.transcribingStreams.append(stream.srcId)
            if doSpeakerRecognition:
                stream.getSpeaker()
            if not doTranscription and not doSpeakerRecognition:
                rospy.logdebug("Stream {} too short to process".format(stream.srcId))
            self.sendMessages(stream)
            rospy.logdebug("{} -> {}: {}".format(stream.srcId,
                                                 stream.speaker if stream.hasKnownSpeaker() else "<Unknown Speaker>",
                                                 stream.transcription if stream.hasTranscription() else "<Unknown Transcript>"))

    def sendMessages(self, stream):
        publisher = None
        message = None
        tNow = rospy.Time.now()
        if stream.hasTranscription() and stream.hasKnownSpeaker():
            publisher = self.transcriptionPublisher
            # Send the final transcript message
            message = SentenceTranscription()
            message.sentence_id = stream.sentenceId
            message.stream_id = stream.srcId
            message.frame_ids = stream.seqIds
            message.start = stream.start
            message.end = stream.end
            message.duration = stream.duration
            message.speaker = stream.speaker
            message.sentence = str(stream.transcription)
        elif stream.hasKnownSpeaker():
            publisher = self.speakerPublisher
            # Send intermediate message
            message = Speaker()
            message.stream_id = stream.srcId
            message.frame_ids = stream.seqIds
            message.start = stream.start
            message.end = stream.end if stream.end is not None else stream.start
            message.active = stream.end is None
            message.duration = stream.duration
            message.speaker = stream.speaker
            message.since_last_update = tNow - stream.lastUpdate
            message.azimuth = reduce(lambda x, y: x + y, stream.azimuths) / len(stream.azimuths)
            stream.lastUpdate = tNow

        if publisher is not None:
            addHeader(message)
            try:
                rospy.logdebug("Sending ros topic\n'{}'".format(str(message)))
                publisher.publish(message)
            except rospy.ROSSerializationException as e:
                rospy.logerr("Unable to send message: {}".format(e))
                raise

    def cleanStreams(self):
        # Remove all streams that are no longer needed
        # This takes in the region of ~1ms so shouldn't block the ROS event thread majorly
        with self.streamLock:
            # Clean up all enrollment streams if we are no longer in enroll mode
            notFinishedStreams = dict(
                (streamId, stream) for streamId, stream in self.srcStreams.iteritems() if
                not stream.sentMessage and stream.type == StreamType.RECOGNITION and self.action == Actions.Recognise)

            if len(self.srcStreams) != len(notFinishedStreams):
                cleanedIds = [str(streamId) for streamId in self.srcStreams.iterkeys() if
                              streamId not in notFinishedStreams.keys()]
                rospy.logdebug("Cleaning up streams {}".format(','.join(cleanedIds)))

            self.srcStreams = notFinishedStreams

    def enroll(self, req):
        rospy.logdebug("Attempting start of enrollment {}".format(req.name))
        self.enrollName = req.name
        self._savedAction = self.action
        self.action = Actions.Enroll
        rospy.loginfo("Beginning enrollment of {}".format(req.name))
        return True

    def endEnroll(self, req):
        rospy.logdebug("Attempting end of enrollment")
        if self.action != Actions.Enroll:
            rospy.logerr("Cannot end enrollment when none is in process")
            return False
        self.checkEnrollStream()
        self.numAdded += 1
        self.action = self._savedAction
        self.trained = False
        self.sendParticipantMessage(self.enrollName)
        rospy.loginfo("Completed enrollment of {}".format(self.enrollName))
        return True

    def startRecognition(self, req):
        rospy.logdebug("Attempting to start recognition")
        self.action = Actions.Recognise
        rospy.loginfo("Running")
        return True

    def saveModel(self, req):

        if not SAVELOADEVENTSUPPORTED:
            rospy.logerr("Saving and loading models is currently not supported")
            return False

        rospy.logdebug("Attempting to save model as {}".format(req.name))
        if len(self.recogniser.features) == 0:
            rospy.logerr("Cannot save model with not enrolled people")
            return False
        if not self.trained:
            rospy.logerr("Cannot save an untrained model")
            return False
        self.recogniser.dump(req.name)
        rospy.loginfo("Save model as {}".format(req.name))
        return True

    def loadModel(self, req):

        if not SAVELOADEVENTSUPPORTED:
            rospy.logerr("Saving and loading models is currently not supported")
            return False

        return self._loadModel(req.name)

    def _loadModel(self, name):

        rospy.logdebug("Attempting to load model as {}".format(name))
        try:
            self.recogniser = SpeakerRecognizer.load(name)
            rospy.loginfo(
                "Loaded speaker model file '{}' with {} features".format(name, len(self.recogniser.features)))
        except IOError:
            rospy.logerr("No model file '{}' found for existing speakers.".format(name))
            return False

        for p in self.recogniser.features.iterkeys():
            self.sendParticipantMessage(p)

        rospy.loginfo("Completed loading model file '{}'".format(name))

        return True

    def sendParticipantMessage(self, name):
        rospy.logdebug("Sending participant message {}".format(name))
        msg = AddedUser()
        addHeader(msg)
        msg.name = name
        self.userAddedPublisher.publish(msg)

    def sendStart(self):
        msg = StartRecognitionMsg()
        addHeader(msg)
        msg.start = True
        self.startedPublisher.publish(msg)

    def start(self):
        self.rpcDispatcher.start()

        rospy.loginfo("Starting")

        rate = rospy.Rate(1)
        firstLoop = True
        while not rospy.is_shutdown():
            rate.sleep()
            if self.action == Actions.Recognise:
                if firstLoop:
                    if self.numAdded > 0:
                        self.recogniser.train()
                    self.sendStart()
                    firstLoop = False
                self.checkStreams()
                self.cleanStreams()

            elif self.action == Actions.Enroll:
                self.checkEnrollStream()

        rospy.loginfo("Stopping")

        if self.numAdded > 0:
            fn = "models/{}_{}.bin".format('_'.join(self.recogniser.features.keys()), datetime.date.today())
            rospy.loginfo("Saving model as {}".format(fn))
            self.recogniser.dump(fn)


if __name__ == '__main__':
    node = Node(modelFile=SPEAKERMODEL, debugLevel=rospy.INFO)
    try:
        node.start()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted")
