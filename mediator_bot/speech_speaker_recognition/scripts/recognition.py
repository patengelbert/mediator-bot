#!/usr/bin/env python

import numpy as np
import rospy
from hark_msgs.msg import HarkSrcWave
from threading import Lock

# Change this to config file
FRAMELENGTH = 512
SAMPLERATE = 16000
DATATYPE = np.int16


class RecogniserNodeException(Exception):
    def __init__(self, what="Exception in Recogniser Node"):
        self.what = what

    def __str__(self):
        return self.what


class InvalidSrcException(RecogniserNodeException):
    pass


class SrcStreamEndedException(RecogniserNodeException):
    pass


class SrcStream(object):
    def __init__(self, srcId, sampleRate):
        self.sampleRate = sampleRate
        self.attachedLabel = None
        self.srcId = srcId
        self.data = []
        self.seqIds = []  # List of seqId, startSample, endSample
        self.lock = Lock()
        self.active = True

    def inUse(self):
        return self.lock.locked()

    def addFrame(self, seqId, src):
        if src.id != self.srcId:
            raise InvalidSrcException(
                "Cannot attach frame from source stream {} to frames of source stream {}".format(src.id, self.srcId))
        if not self.active:
            raise SrcStreamEndedException("Cannot attach frame to ended source stream {}".format(self.srcId))

        if src is None:
            self.endStream()
        else:
            with self.lock:
                self.seqIds.append((seqId, len(self.data), len(self.data) + src.length - 1))
                self.data += src.wavedata

    def endStream(self):
        if not self.active:
            raise SrcStreamEndedException("Stream {} has already been ended".format(self.srcId))
        self.active = False
        # End of data for this stream
        rospy.loginfo("End of data for stream {} after {}s".format(self.srcId, self.duration))

    @property
    def length(self):
        return len(self.data)

    @property
    def duration(self):
        return float(self.length) / float(self.sampleRate)


class Node(object):
    def __init__(self):
        self.srcStreams = {}
        self.activeStreams = set()

    def addToStreams(self, data):
        if data.exist_src_num == 0:
            for streamId in self.activeStreams:
                self.srcStreams[streamId].endStream()
            self.activeStreams.clear()
        else:
            updatedStreams = set()
            for srcStream in data.src:
                streamId = srcStream.id
                if streamId not in self.srcStreams.keys():
                    self.srcStreams[streamId] = SrcStream(streamId, SAMPLERATE)
                    self.activeStreams.add(streamId)
                self.srcStreams[streamId].addFrame(data.header.seq, srcStream)
                updatedStreams.add(streamId)

            for streamId in [sid for sid in self.activeStreams if sid not in updatedStreams]:
                self.activeStreams.remove(streamId)
                self.srcStreams[streamId].endStream()

    def listener(self):
        rospy.init_node('SpeakerSpeechNode')

        rospy.Subscriber("HarkSrcWave", HarkSrcWave, self.addToStreams)

        rospy.loginfo("Running")

        rospy.spin()


if __name__ == '__main__':
    node = Node()
    node.listener()
