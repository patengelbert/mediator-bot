#!/usr/bin/env python

import numpy as np
import rospy
from concurrent.futures import TimeoutError
from speaker_recognition.exceptions import FeatureExtractionException

from custom_exceptions import (
    UnknownObjectError,
    InvalidSrcException,
    SrcStreamEndedException,
    SpeakerRecognitionError,
    SrcStreamStillActiveException,
    TranscriptionError,
    EnrollmentError)
from speech_recognition import AudioData

import base64

from locks import TimeoutLock

from config import (
    MINDIARIZELENGTH,
    SAMPLERATE,
    TIMEOUT,
    MINTRANSCRIBELENGTH
)


class _ErrorTranscript(object):
    def __str__(self):
        return "<Error Fetching Transcript>"

    def __eq__(self, other):
        raise UnknownObjectError("Cannot compare unknown transcript")


class _ErrorSpeaker(object):
    def __str__(self):
        return "<Error Fetching Speaker>"

    def __eq__(self, other):
        raise UnknownObjectError("Cannot compare unknown speaker")


errorTranscript = _ErrorTranscript()
errorSpeaker = _ErrorSpeaker()


class StreamType:
    Unknown = 0
    Enrollment = 1
    Recognition = 2


class SrcStream(object):
    def __init__(self, srcId, sampleRate, rpcDispatcher, speakerRecogniser, startTime, streamType=StreamType.Recognition, speaker=None):
        self.rpcDispatcher = rpcDispatcher
        self.sampleRate = sampleRate
        self._speaker = speaker
        self.srcId = srcId
        self.data = []
        self.seqIds = []  # List of seqId, startSample, endSample
        self.active = True
        self._transcript = None
        self.recogniser = speakerRecogniser

        self.speakerLock = TimeoutLock()
        self.dataLock = TimeoutLock()
        self.transcriptionLock = TimeoutLock()

        self.transcriptFuture = None

        self.start = startTime
        self.end = None

        self.type = streamType

    def addFrame(self, seqId, timeStamp, src):
        if src.id != self.srcId:
            raise InvalidSrcException(
                "Cannot attach frame from source stream {} to frames of source stream {}".format(src.id, self.srcId))
        if not self.active:
            raise SrcStreamEndedException("Cannot attach frame to ended source stream {}".format(self.srcId))

        if src is None:
            self.endStream(timeStamp)
        else:
            with self.dataLock:
                self.seqIds.append(seqId)
                self.data += src.wavedata

    def endStream(self, timeStamp):
        if not self.active:
            raise SrcStreamEndedException("Stream {} has already been ended".format(self.srcId))
        self.end = timeStamp
        self.active = False
        # End of data for this stream
        rospy.loginfo("End of data for stream {} after {}s".format(self.srcId, self.duration))

    def hasKnownSpeaker(self):
        return self._speaker is not None

    @property
    def speaker(self):
        return self.getSpeaker()

    @speaker.setter
    def speaker(self, val):
        self._speaker = val

    def getSpeaker(self, allowShortDuration=False, forceRetry=False):
        with self.speakerLock:
            if self._speaker is not None and not forceRetry:
                return self._speaker

            if not allowShortDuration and self.duration < MINDIARIZELENGTH:
                raise SpeakerRecognitionError(
                    "Stream {} is too short to recognise the speaker. {} < {}".format(self.srcId, self.duration,
                                                                                      MINDIARIZELENGTH))

            rospy.loginfo("Beginning speaker recognition for stream {}".format(self.srcId))

            try:
                with self.dataLock:
                    audioData = self.getAudioData()
                self._speaker = self.recogniser.predict(audioData)
            except FeatureExtractionException as e:
                rospy.logerr(e)
                self._speaker = errorSpeaker
            rospy.loginfo("Completed speaker recognition for stream {}".format(self.srcId))
            return self._speaker

    def getAudioData(self):
        return AudioData(np.array(self.data, np.int16).tostring(), SAMPLERATE, 2)

    def hasTranscription(self):
        return self._transcript is not None

    def doTranscription(self, forceRetry=False, async=False):
        self.transcriptionLock.acquire()
        if self._transcript is not None and not forceRetry:
            self.transcriptionLock.release()
            return self._transcript
        else:
            if self.active:
                self.transcriptionLock.release()
                raise SrcStreamStillActiveException(
                    "Stream {} cannot be transcribed while it is still active".format(self.srcId))
            if self.duration < MINTRANSCRIBELENGTH:
                self.transcriptionLock.release()
                raise TranscriptionError(
                    "Stream {} is too short to transcribe {} < {}".format(self.srcId, self.duration,
                                                                          MINTRANSCRIBELENGTH))

            rospy.loginfo("Beginning transcription for stream {}".format(self.srcId))

            with self.dataLock:
                audioData = self.getAudioData()

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
            self.transcriptFuture = transcript
            if async:
                return
            return self.waitForTranscription()

    def checkTranscription(self):
        if self.transcriptFuture.done():
            # This won't block anymore
            return self.waitForTranscription()
        return None

    def waitForTranscription(self):
        try:
            rv = self.transcriptFuture.result(timeout=TIMEOUT)
            if len(rv.get('results', [])) > 0 and len(rv['results'][0].get('alternatives', [])) > 0:
                self._transcript = rv['results'][0]['alternatives'][0]['transcript']
            else:
                self._transcript = errorTranscript
        except Exception as e:
            if isinstance(e, TimeoutError):
                rospy.logerr("Timed out waiting for result of transcription for stream {}".format(self.srcId))
            else:
                rospy.logerr("Error fetching transcript for stream {}: {}".format(self.srcId, e))
            self._transcript = errorTranscript
        self.transcriptionLock.release()
        rospy.loginfo("Completed transcription for stream {}".format(self.srcId))
        return self._transcript

    @property
    def transcription(self):
        return self.doTranscription()

    def __len__(self):
        return self.length

    @property
    def length(self):
        return len(self.data)

    @property
    def duration(self):
        return self.length / float(self.sampleRate)

    @property
    def sentMessage(self):
        return not self.active and self.hasKnownSpeaker() and self.hasTranscription()

    def enroll(self, allowShortDuration=False):
        if self.active:
            raise EnrollmentError("Stream {} cannot be enrolled while it is still active".format(self.srcId))

        if not allowShortDuration and self.duration < MINDIARIZELENGTH:
            raise EnrollmentError(
                "Stream {} is too short to enroll the speaker. {} < {}".format(self.srcId, self.duration,
                                                                                  MINDIARIZELENGTH))

        rospy.loginfo("Beginning speaker enrollment for {}, stream {}".format(self._speaker, self.srcId))

        try:
            with self.dataLock:
                audioData = self.getAudioData()
            self.recogniser.enroll(audioData, self._speaker)
        except FeatureExtractionException as e:
            rospy.logerr(e)
        rospy.loginfo("Completed speaker enrollment for {}, stream {}".format(self._speaker, self.srcId))
