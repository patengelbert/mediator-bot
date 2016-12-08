#!/usr/bin/env python
import threading

import rospy
import std_msgs.msg
from hark_msgs.msg import HarkSrcWave
from speaker_recognition import SpeakerRecognizer
from speech_speaker_recognition.msg import SentenceTranscription, Speaker
from speech_speaker_recognition.srv import StartEnrollment, EndEnrollment, StartRecognition
from config import (
    SPEAKERMODEL,
    MINTRANSCRIBELENGTH,
    MINDIARIZELENGTH,
    SAMPLERATE,

)
from rpc_dispatcher import RPCDispatcher
from src_stream import SrcStream, StreamType


class Actions:
    Unknown = 0
    Recognise = 1
    Enroll = 2
    Starting = 3
    Stopping = 4


actionNames = {
    Actions.Unknown: "Unknown",
    Actions.Recognise: "Recognise",
    Actions.Enroll: "Enroll",
    Actions.Starting: "Starting",
    Actions.Stopping: "Stopping",
}


class Node(object):
    def __init__(self):
        self._action = Actions.Starting
        self.srcStreams = dict()
        self.activeStreams = set()
        self.streamLock = threading.Lock()  # Used to prevent all streams from being modified while processing is occurring
        self.actionLock = threading.Lock()

        self.rpcDispatcher = RPCDispatcher()
        self.recogniser = SpeakerRecognizer()
        self.model = SPEAKERMODEL
        self.transcribingStreams = []

        self.transcriptionPublisher = rospy.Publisher('transcriptions', SentenceTranscription, queue_size=10)
        self.speakerPublisher = rospy.Publisher('speaker', Speaker, queue_size=10)

        self.enrollName = None
        self._savedAction = None
        self.numAdded = 0

    @property
    def action(self):
        with self.actionLock:
            return self._action

    @action.setter
    def action(self, val):
        with self.actionLock:
            rospy.loginfo("Changing mode to {}".format(actionNames[val]))
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
                        streamType = StreamType.Enrollment if self.action == Actions.Enroll else StreamType.Recognition
                        self.srcStreams[streamId] = SrcStream(streamId, SAMPLERATE, self.rpcDispatcher, self.recogniser,
                                                              data.header.stamp, streamType=streamType,
                                                              speaker=self.enrollName if streamType == StreamType.Enrollment else None)
                        self.activeStreams.add(streamId)

                    self.srcStreams[streamId].addFrame(data.header.seq, data.header.stamp, srcStream)
                    updatedStreams.add(streamId)

                for streamId in [sid for sid in self.activeStreams if sid not in updatedStreams]:
                    self.activeStreams.remove(streamId)
                    self.srcStreams[streamId].endStream(data.header.stamp)

    def checkEnrollStream(self):
        with self.streamLock:
            streamsToCheck = [stream for stream in self.srcStreams.itervalues() if stream.type == StreamType.Enrollment]

            for stream in streamsToCheck:
                if not stream.active and not stream.enrolled:
                    stream.enroll()

    def checkStreams(self):
        with self.streamLock:
            streamsToCheck = [stream for stream in self.srcStreams.itervalues() if
                              stream.type == StreamType.Recognition and
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
        if stream.hasTranscription() and stream.hasKnownSpeaker():
            publisher = self.transcriptionPublisher
            # Send the final transcript message
            message = SentenceTranscription()
            message.header = std_msgs.msg.Header()
            message.header.stamp = rospy.Time.now()
            message.sentence_id = 0  # TODO actually figure out what the sentence ids are
            message.stream_id = stream.srcId
            message.frame_ids = stream.seqIds
            message.start = stream.start
            message.end = stream.end
            message.duration = stream.duration
            message.speaker = stream.speaker
            message.sentence = stream.transcription
        elif stream.hasKnownSpeaker():
            publisher = self.speakerPublisher
            # Send intermediate message
            # Send the final transcript message
            message = Speaker()
            message.header = std_msgs.msg.Header()
            message.header.stamp = rospy.Time.now()
            message.stream_id = stream.srcId
            message.frame_ids = stream.seqIds
            message.start = stream.start
            message.end = stream.end if stream.end is not None else stream.start
            message.active = stream.end is not None
            message.duration = stream.duration
            message.speaker = stream.speaker
        if publisher is not None:
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
                not stream.sentMessage and stream.type == StreamType.Recognition and self.action == Actions.Recognise)

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
        rospy.logdebug("Completed enrollment")
        return True

    def startRecognition(self, req):
        rospy.logdebug("Attempting to start recognition")
        self.action = Actions.Recognise
        rospy.loginfo("Running")
        return True

    def start(self):
        self.rpcDispatcher.start()
        rospy.init_node('SpeakerSpeechNode')
        rospy.on_shutdown(self.rpcDispatcher.stop)

        if self.model is not None:
            try:
                self.recogniser = SpeakerRecognizer.load(self.model)
                rospy.loginfo("Loaded speaker model {}".format(self.model))
            except IOError:
                rospy.logwarn("No model file '{}' found for existing speakers.".format(self.model))
                rospy.logwarn("Please enroll people before starting detection")

        rospy.Subscriber("HarkSrcWave", HarkSrcWave, self.addToStreams)

        rospy.Service('StartEnrollment', StartEnrollment, self.enroll)
        rospy.Service('EndEnrollment', EndEnrollment, self.endEnroll)
        rospy.Service('StartRecognition', StartRecognition, self.startRecognition)

        rospy.loginfo("Starting")

        rate = rospy.Rate(1)

        firstLoop = True
        while not rospy.is_shutdown():
            rate.sleep()
            if self.action == Actions.Recognise:

                if firstLoop and self.numAdded >= 1:
                    # It crashes when in service
                    self.recogniser.train()
                    rospy.loginfo("Trained GMM for {} new speakers".format(self.numAdded))
                firstLoop = False

                self.checkStreams()
                self.cleanStreams()
            elif self.action == Actions.Enroll:
                self.checkEnrollStream()


if __name__ == '__main__':
    node = Node()
    try:
        node.start()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted")
