#!/usr/bin/env python

import rospy
import sys

from enum import Enum
from speech_speaker_recognition.msg import AddedUser, Speaker
from mediator_bot_msgs.msg import MedBotSpeechTiming, MedBotSpeechStatus
from mediator_bot_msgs.srv import MedBotSpeechQuery

INC_FACTOR_POS = 2
INC_FACTOR_NEG = 5
DEC_FACTOR_POS = 0.5
DEC_FACTOR_NEG = 0.1
MAX_POS = 10
MIN_NEG = -10
THRESHOLD_POS = 5
THRESHOLD_NEG = -5
START_WEIGHT = 0
RATE = 10

CURSOR_UP_ONE = '\x1b[1A'
ERASE_LINE = '\x1b[2K'
SPEAKING_SYMBOL = unichr(0x265B)
BAR = unichr(0x2588)
EMPTY_BAR = '-'


class Status(Enum):
    NO_STATUS = 0
    TOO_LONG = 1
    TOO_SHORT = 2


class SpeakerContainer(object):
    pub = None

    def __init__(self, label, pub, startWeight=START_WEIGHT, maxVal=MAX_POS, minVal=MIN_NEG, thresholdPos=THRESHOLD_POS,
                 thresholdNeg=THRESHOLD_NEG):

        self.pub = pub

        self._weight = startWeight
        self.speaking = False
        self.label = label
        self._status = Status.NO_STATUS

        self.maxVal = maxVal
        self.minVal = minVal

        self.thresholdPos = thresholdPos
        self.thresholdNeg = thresholdNeg

        self.azimuths = []

    @property
    def msg(self):
        # noinspection PyUnresolvedReferences
        return self.status.name

    @property
    def status(self):
        return self._status

    @status.setter
    def status(self, val):
        if self._status != val:
            self.sendMessage()
            rospy.logdebug("{}: Setting status - {}".format(self.label, val))
        self._status = val

    @property
    def weight(self):
        return self._weight

    @weight.setter
    def weight(self, val):
        self._weight = min(max(val, self.minVal), self.maxVal)

        if self._weight > self.thresholdPos:
            self.status = Status.TOO_LONG
        elif self._weight < self.thresholdNeg:
            self.status = Status.TOO_SHORT
        else:
            self.status = Status.NO_STATUS

    def sendMessage(self):
        msg = MedBotSpeechTiming()
        msg.header = rospy.Header()
        msg.header.stamp = rospy.Time.now()
        msg.weight = self.weight
        msg.status = self.status
        msg.speaking = self.speaking
        msg.azimuth = reduce(lambda x, y: x + y, self.azimuths) / len(self.azimuths)
        self.pub.publish(msg)


class TimeAllocator:
    """
    Module assigning a weighting to each speaker to show their current priority
    in the conversation, relative to all the other participants.

    The participants are identified by the direction they are sitting from the
    recording devices. This naive approach may be changed.
    """

    def callback(self, data):
        """
        Callback function to show who is currently speaking
        """
        speaker = self.speakers.get(data.speaker)
        if speaker is None:
            rospy.logerr("Speaker {} has not yet been registered".format(data.speaker))
            return

        speaker.speaking = data.active
        speaker.azimuths.append(data.azimuth)

    def getSpeechStatus(self, req):
        speaker = self.speakers.get(req.name)
        if speaker is None:
            rospy.logerr("Speaker {} cannot be found".format(req.speaker))
            return None
        return dict(
            name=req.name,
            weight=speaker.weight,
            status=speaker.status,
            speaking=speaker.speaking,
        )

    def userAdded(self, data):
        rospy.loginfo("Adding {}".format(data.name))
        self.speakers[data.name] = SpeakerContainer(data.name, self.pub)

    def __init__(self, rate=RATE, debugLevel=rospy.INFO):
        self.debugLevel = debugLevel

        rospy.init_node('TimeAllocator', anonymous=True, log_level=debugLevel)

        rospy.loginfo("Initialising TimeAllocator node")

        # time allocation containers
        self.speakers = dict()

        # Flag. Module in init stage
        self.init = True

        # Create subscribers/publishers
        rospy.Subscriber("added_user", AddedUser, self.userAdded)
        rospy.Subscriber("speaker", Speaker, self.callback)
        self.pub = rospy.Publisher('/speaker_change_state', MedBotSpeechStatus, queue_size=10, latch=True)
        self.pubWeight = rospy.Publisher('/speaker_weightings', MedBotSpeechTiming, queue_size=10, latch=True)

        rospy.Service('/query_speaker_state', MedBotSpeechQuery, self.getSpeechStatus)

        self.rate = rate

    def printMultProgress(self, barLength=50):
        """
        Debug function.
        Source: http://stackoverflow.com/a/34325723

        Call in a loop to create terminal progress bar
        @params:
            barLength   - Optional  : character length of bar (Int)
        """

        if not self.init:
            sys.stdout.write(''.join([CURSOR_UP_ONE] * len(self.speakers)))
        self.init = False

        for speaker in self.speakers.itervalues():
            percent = 100 * float(speaker.weight) / \
                      (float(speaker.maxVal) if speaker.weight >= 0 else (-1 * float(speaker.minVal)))
            filledLength = int(round(barLength * max(speaker.weight, 0) / float(speaker.maxVal)))
            bar = BAR * filledLength + EMPTY_BAR * (barLength - filledLength)

            s = ERASE_LINE + u"{:8.8s}{:1s} {: 3.1f} |{}| {: .1f}% {}\n".format(speaker.label,
                                                                                SPEAKING_SYMBOL if speaker.speaking else "",
                                                                                speaker.weight, bar, percent,
                                                                                speaker.msg)
            sys.stdout.write(s)

        sys.stdout.flush()

    def calc(self):
        """
        Main function for calculating the current time allocation for each
        speaker. The result is printed on the terminal.
        """
        # Increment/decrement the time allocation based on if they are speaking
        for speaker in self.speakers.itervalues():
            if speaker.speaking:
                factor = INC_FACTOR_POS if speaker.weight >= 0 else INC_FACTOR_NEG
                speaker.weight += float(factor) / float(self.rate)
            else:
                factor = DEC_FACTOR_POS if speaker.weight >= 0 else DEC_FACTOR_NEG
                speaker.weight -= float(factor) / float(self.rate)
        if self.debugLevel <= rospy.INFO:
            # Print out the current speech participation levels
            self.printMultProgress()

        self.packMsg()

    def packMsg(self):
        msg = MedBotSpeechTiming()
        msg.header = rospy.Header()
        msg.header.stamp = rospy.Time().now()
        msg.num_speakers = len(self.speakers)
        msg.speaker_id = [s.label for s in self.speakers.itervalues()]
        msg.weighting = [float(s.weight) for s in self.speakers.itervalues()]
        self.pubWeight.publish(msg)

    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.calc()
            r.sleep()


if __name__ == '__main__':
    t = TimeAllocator(debugLevel=rospy.INFO)
    t.run()
