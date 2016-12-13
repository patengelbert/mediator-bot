#! /usr/bin/env python
import threading

import rospy
import random
import smach
import actionlib
from enum import Enum
from mediator_bot_msgs.msg import MedBotSpeechStatus
from speech_speaker_recognition.msg import AddedUser, StartRecognitionMsg

from action_response.msg import responseAction, responseGoal


class Status(Enum):
    NO_STATUS = 0
    TOO_LONG = 1
    TOO_SHORT = 2


class Speaker(object):
    def __init__(self, label, azimuth=0.0, weight=0.0, speaking=False):
        rospy.logdebug("Adding speaker {}".format(label))
        self.label = label
        self.azimuth = azimuth
        self.weight = weight
        self.speaking = speaking
        self.status = Status.NO_STATUS


class SpeakerStates:
    """
    Container for current speaker states
    """
    init = False
    speakers = {}

    def addNewSpeaker(self, data):
        self.speakers[data.name] = Speaker(data.name)

    def updateSpeaker(self, data):
        rospy.logdebug("Updating speaker {} -> {}".format(data.name, Status(data.status)))
        speaker = self.speakers.get(data.name, None)
        if speaker is None:
            self.addNewSpeaker(data)
            speaker = self.speakers[data.name]
        speaker.azimuth = data.azimuth
        speaker.weight = data.weight
        speaker.status = Status(data.status)
        speaker.speaking = data.speaking

    def getTooLongSpeakers(self):
        return [s for s in self.speakers.itervalues() if s.status == Status.TOO_LONG]

    def getTooShortSpeakers(self):
        return [s for s in self.speakers.itervalues() if s.status == Status.TOO_SHORT]

    def getNextTooShortSpeaker(self):
        s = self.getTooShortSpeakers()
        if len(s) == 0:
            return None
        return random.choice(s)

    def getNextTooLongSpeaker(self):
        s = self.getTooLongSpeakers()
        if len(s) == 0:
            return None
        return random.choice(s)

    def getActiveSpeakers(self):
        return [s for s in self.speakers.itervalues() if s.speaking]

    def getNumActiveSpeakers(self):
        return len(self.getActiveSpeakers())

    def getLowestWeightedSpeaker(self):
        l = sorted(self.speakers.values(), key=lambda x: x.weight)
        return l[0] if len(l) > 0 else None

    def getHighestWeightedSpeaker(self):
        l = sorted(self.speakers.values(), key=lambda x: x.weight, reverse=True)
        return l[0] if len(l) > 0 else None


class ActionClient(object):
    client = None

    def __init__(self):
        self.client = actionlib.SimpleActionClient('response', responseAction)
        self.client.wait_for_server()

    def req(self, name, direction, keywords):
        goal = responseGoal(keywords=keywords, name=name, direction=direction)
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))


running = False


class State(smach.State):
    outcomes = []

    def __init__(self, speakerStates, client):
        self.speakerStates = speakerStates
        self.client = client
        smach.State.__init__(self, outcomes=self.outcomes + ['errored', 'preempted'])

    def execute(self, userdata):
        try:
            rospy.loginfo("Executing state {:s}".format(self))
            return self._execute(userdata)
        except Exception as e:
            rospy.logerr(e)
            return None  # Kill thread

    def _execute(self, userdata):
        raise NotImplementedError

    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        smach.State.request_preempt(self)
        rospy.logwarn("Preempted!")

    def checkPreemption(self):
        if self.preempt_requested():
            self.service_preempt()
            return None  # Kill thread

    def __str__(self):
        return self.__class__.__name__


# States #

# define state Initialise
class Initialise(State):
    """
    Initialise the module and wait to start.
    """

    outcomes = ['initialised']

    def _execute(self, userdata):
        # get names from topic, added user
        rospy.loginfo('Waiting for registered users...')
        while not self.speakerStates.speakers and not running:  # wait while there are no registered
            self.checkPreemption()
            rospy.sleep(0.5)
        rospy.loginfo('Users registered')
        return 'initialised'


# define state Start
class StartTopic(State):
    """
    Start the current topic
    """

    outcomes = ['intro_complete']

    def _execute(self, userdata):
        self.introSequence()
        return 'intro_complete'

    def introSequence(self):
        rospy.loginfo('*Intro Sequence*')
        self.client.req(keywords=["intro"], name="", direction=0.0)


class LookAtSpeaker(State):

    outcomes = ['looked']

    def _execute(self, userdata):
        s = []
        while not True:
            s = self.speakerStates.getActiveSpeakers()
            if len(s) > 0:
                break
            rospy.sleep(0.1)
        if len(s) > 0:
            speaker = random.choice(s)
            self.client.req(keywords=['natural'], name=speaker.label, direction=speaker.azimuth)
        return 'looked'


# # define state Mediate
# class Mediate(State):
#     """
#     Main state mediating the conversation. Constantly checking the state of
#     the conversation in order to give a appropriate response.
#     """
#
#     outcomes = ['timeup', 'control_conv', 'not_speaking']
#
#     def __init__(self, speakerStates, client):
#         super(Mediate, self).__init__(speakerStates, client)
#         self.timeup = False
#         self.init = False
#         self.duration = 500
#         self.timerStart = 0
#
#     def _execute(self, userdata):
#         rospy.sleep(3)
#         self.checkPreemption()
#         if not self.init:
#             self.initTimer()
#         while not rospy.is_shutdown():
#             if (rospy.Time.now() - self.timerStart) > rospy.Duration(self.duration - 30) and not self.timeup:
#                 self.client.req(keywords=["nearly_done"], direction=0.0, name="")
#             elif self.timeup:
#                 # time run out
#                 return 'timeup'
#             elif len(self.speakerStates.getTooLongSpeakers()) > 0:
#                 # someones talking too much, ask least talkative person a question
#                 return 'not_speaking'
#             elif self.speakerStates.getNumActiveSpeakers() > 2:
#                 # too many speakers
#                 return 'control_conv'
#
#     def initTimer(self):
#         rospy.Timer(rospy.Duration(self.duration), self.callback, oneshot=True)
#         self.init = True
#         self.timerStart = rospy.Time.now()
#
#     def callback(self, event):
#         rospy.loginfo('Time is up: ' + str(event.current_real))
#         self.timeup = True
#
#
# # define state Quieten
# class Quieten(State):
#     """
#     Too many people are currently talking. Attempt to return to a single
#     speaker.
#     """
#     outcomes = ['success', 'failed']
#
#     def _execute(self, userdata):
#         # Ask to be quiet with nao
#         self.client.req(keywords=["polite", "multiple"], name="", direction=0.0)
#         return 'success' if self.speakerStates.getNumActiveSpeakers() <= 1 else 'failed'
#
#
# # define state Happy
# class Happy(State):
#     """
#     Show the previous action was successful.
#     """
#
#     outcomes = ['success']
#
#     def _execute(self, userdata):
#         rospy.loginfo('Executing state Happy')
#         self.client.req(keywords=["thanks"], name="", direction=0.0)
#         return 'success'
#
#
# # define state AskQuestion
# class AskQuestion(State):
#     """
#     Ask least talkative person a question if someone talking too much.
#     """
#
#     outcomes = ['success']
#
#     def _execute(self, userdata):
#         # direct question at person
#         speaker = self.speakerStates.getNextTooLongSpeaker()
#         if speaker is not None:
#             self.client.req(keywords=["stop"], name=speaker.label,
#                             direction=speaker.azimuth)
#         return 'success'
#
#
# # define state Start
# class CloseTopic(State):
#     """
#     Wrap up the current topic.
#     """
#
#     outcomes = ['finished', 'next_topic']
#
#     def _execute(self, userdata):
#         rospy.sleep(1)
#         self.checkPreemption()
#         self.client.req(keywords=["outro"], name="", direction=0.0)
#         return 'finished'


# def callbackLoud(): # too loud
# Compute average power of the frames and make sure it doesnt exceed a threshold

# Status checking functions #

def started(data):
    global running
    running = True


def main():
    rospy.init_node('mediatorbot_state_machine', log_level=rospy.DEBUG)

    speakerStates = SpeakerStates()
    # actionclient = ActionClient()
    actionclient = None
    rospy.Subscriber("start_recognition", StartRecognitionMsg, started)
    rospy.Subscriber("/added_user", AddedUser, speakerStates.addNewSpeaker)
    rospy.Subscriber("/speaker_change_state", MedBotSpeechStatus,
                     speakerStates.updateSpeaker)  # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end', 'preempted', 'errored'])

    with sm:
        # Add states to the container
        smach.StateMachine.add('Initialise', Initialise(speakerStates, actionclient),
                               transitions={'initialised': 'LookAtSpeaker',
                                            'preempted': 'preempted',
                                            'errored': 'errored'})
        smach.StateMachine.add('LookAtSpeaker', LookAtSpeaker(speakerStates, actionclient),
                               transitions={'looked': 'LookAtSpeaker',
                                            'preempted': 'preempted',
                                            'errored': 'errored'})
        # smach.StateMachine.add('StartTopic', StartTopic(speakerStates, actionclient),
        #                        transitions={'intro_complete': 'Mediate',
        #                                     'preempted': 'preempted',
        #                                     'errored': 'errored'})
        # smach.StateMachine.add('Mediate', Mediate(speakerStates, actionclient),
        #                        transitions={'timeup': 'CloseTopic', 'control_conv': 'Quieten',
        #                                     'not_speaking': 'AskQuestion',
        #                                     'preempted': 'preempted',
        #                                     'errored': 'errored'})
        # smach.StateMachine.add('Quieten', Quieten(speakerStates, actionclient),
        #                        transitions={'success': 'Happy', 'failed': 'Mediate',
        #                                     'preempted': 'preempted',
        #                                     'errored': 'errored'})
        # smach.StateMachine.add('Happy', Happy(speakerStates, actionclient),
        #                        transitions={'success': 'Mediate',
        #                                     'preempted': 'preempted',
        #                                     'errored': 'errored'})
        # smach.StateMachine.add('AskQuestion', AskQuestion(speakerStates, actionclient),
        #                        transitions={'success': 'Mediate',
        #                                     'preempted': 'preempted',
        #                                     'errored': 'errored'})
        # smach.StateMachine.add('CloseTopic', CloseTopic(speakerStates, actionclient),
        #                        transitions={'finished': 'end', 'next_topic': 'StartTopic',
        #                                     'preempted': 'preempted',
        #                                     'errored': 'errored'})

    # Execute SMACH plan
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.daemon = True
    smach_thread.start()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sm.request_preempt()

    rospy.logdebug("Waiting for smach to shut down")


if __name__ == '__main__':
    main()
