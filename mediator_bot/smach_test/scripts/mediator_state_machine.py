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


EXTRA_KEYWORD_THRESHOLD = 0.6
SELECT_OTHER_THRESHOLD = 0.5
SAY_THANK_YOU_THRESHOLD = 0.5

SINGLE_TIMEOUT = 10.0
GROUP_TIMEOUT = 60.0
ACTION_TIMEOUT = 25.0

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
        self.timeout = None
        self.allowed = True

    def startTimeout(self, time):
        rospy.logdebug("Starting timeout of {:s}".format(self))
        self.allowed = False
        self.timeout = rospy.Timer(rospy.Duration(time), self._doneTimer, True)

    def _doneTimer(self, *args, **kwargs):
        rospy.logdebug("Finished timeout of {:s}".format(self))
        self.timeout = None
        self.allowed = True

    def cancelTimeout(self):
        if self.timeout is not None:
            rospy.logdebug("Cancelled timeout of {:s}".format(self))
            self.timeout.shutdown()
        self._doneTimer()

    def __str__(self):
        return str(self.label)


class SpeakerStates:
    """
    Container for current speaker states
    """
    init = False
    speakers = {}

    def addNewSpeaker(self, data):
        self.speakers[data.name] = Speaker(data.name)

    def updateSpeaker(self, data):
        speaker = self.speakers.get(data.name, None)
        if speaker is None:
            self.addNewSpeaker(data)
            speaker = self.speakers[data.name]
        speaker.azimuth = data.azimuth
        speaker.weight = data.weight
        speaker.status = Status(data.status)
        speaker.speaking = data.speaking

    def getTooLongSpeakers(self):
        return [s for s in self.speakers.itervalues() if s.status == Status.TOO_LONG and s.allowed]

    def getTooShortSpeakers(self):
        return [s for s in self.speakers.itervalues() if s.status == Status.TOO_SHORT and s.allowed]

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

    def getTooLongActiveSpeakers(self):
        return [s for s in self.speakers.itervalues() if s.speaking and s.status == Status.TOO_LONG and s.allowed]

    def getNextTooLongActiveSpeaker(self):
        l = self.getTooLongActiveSpeakers()
        if len(l) == 0:
            return None
        return random.choice(l)

    def getTooShortInactiveSpeakers(self):
        return [s for s in self.speakers.itervalues() if not s.speaking and s.status == Status.TOO_SHORT and s.allowed]

    def getNextTooShortInactiveSpeaker(self):
        l = self.getTooShortInactiveSpeakers()
        if len(l) == 0:
            return None
        return random.choice(l)


class ActionClientException(Exception):
    def __init__(self, what):
        self.what = what

    def __str__(self):
        return str(self.what)


class ActionClient(object):
    client = None

    allowGroup = True
    timeout = None
    allowSpecialActions = True

    def __init__(self, name="response", timeout=5.0):
        self.topic = name

        self.client = actionlib.SimpleActionClient(self.topic, responseAction)
        rospy.logdebug("Waiting for server")
        if not self.client.wait_for_server(timeout=rospy.Duration(timeout)):
            raise ActionClientException("Could not connect to action server '{}'".format(self.topic))

    def req(self, name, direction, keywords, timeout=5.0):
        goal = responseGoal(keywords=keywords, name=name, direction=direction)
        self.client.send_goal(goal)
        if not self.client.wait_for_result(timeout=rospy.Duration(timeout)):
            raise ActionClientException("Did not receive response from server '{}' for request {:s}".format(self.topic, goal))

    def startTimeout(self, time):
        rospy.logdebug("Starting timeout of groups")
        self.allowGroup = False
        self.timeout = rospy.Timer(rospy.Duration(time), self._doneTimer, True)

    def _doneTimer(self, *args, **kwargs):
        rospy.logdebug("Finished timeout of groups")
        self.timeout = None
        self.allowGroup = True

    def cancelTimeout(self):
        if self.timeout is not None:
            rospy.logdebug("Cancelled timeout of groups")
            self.timeout.shutdown()
        self._doneTimer()

    def startActionTimeout(self, time):
        rospy.logdebug("Starting timeout of groups")
        self.allowSpecialActions = False
        self.timeout = rospy.Timer(rospy.Duration(time), self._doneActionTimer, True)

    def _doneActionTimer(self, *args, **kwargs):
        rospy.logdebug("Finished timeout of groups")
        self.timeout = None
        self.allowSpecialActions = True

    def cancelActionTimeout(self):
        if self.timeout is not None:
            rospy.logdebug("Cancelled timeout of groups")
            self.timeout.shutdown()
        self._doneTimer()


running = False


class State(smach.State):
    outcomes = []
    input_data = []
    output_data = []

    def __init__(self, speakerStates, client):
        self.speakerStates = speakerStates
        self.client = client
        smach.State.__init__(self, outcomes=self.outcomes + ['errored', 'preempted'], input_keys=self.input_data,
                             output_keys=self.output_data)

    def execute(self, userdata):
        try:
            rospy.loginfo("Executing state {:s}".format(self))
            return self._execute(userdata)
        except Exception as e:
            rospy.logerr(e)
            return 'errored'

    def _execute(self, userdata):
        raise NotImplementedError

    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        smach.State.request_preempt(self)
        rospy.logwarn("Preempted!")

    def checkPreemption(self):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

    def req(self, *args, **kwargs):
        try:
            self.client.req(*args, **kwargs)
        except ActionClientException as e:
            rospy.logwarn(e)

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
        while not running:  # wait while there are no registered
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
        self.req(keywords=["intro"], name="", direction=0.0, timeout=10.0)


class LookAtSpeaker(State):
    outcomes = ['looked', 'finished', 'quieten', 'question', 'group_question', 'group_quieten']
    output_data = ['name']

    def _execute(self, userdata):
        s = None

        action = self.checkNoImportantActions(userdata)

        if action is not None:
            self.client.startActionTimeout(ACTION_TIMEOUT)
            return action

        count = 0
        while not rospy.is_shutdown() and count < 10:
            self.checkPreemption()
            s = self.speakerStates.getActiveSpeakers()
            if len(s) > 0:
                break
            rospy.sleep(0.1)
            count += 1

        if not rospy.is_shutdown() and s is not None and len(s) > 0:
            speaker = random.choice(s)
            self.req(keywords=['natural'], name=speaker.label, direction=speaker.azimuth)
            return 'looked'

        if not rospy.is_shutdown():
            return 'looked'

        return 'finished'

    def checkNoImportantActions(self, userData):

        if not self.client.allowSpecialActions:
            return None

        tooLongs = self.speakerStates.getTooLongActiveSpeakers()
        tooShorts = self.speakerStates.getTooShortInactiveSpeakers()
        if len(tooShorts) == len(self.speakerStates.speakers):
            userData.name = ""
            return 'group_question'
        elif len(tooLongs) >= 2:
            userData.name = ""
            return 'group_quieten'
        elif len(tooLongs) > 0:
            userData.name = random.choice(tooLongs).label
            return 'quieten'
        elif len(tooShorts) > 0:
            userData.name = random.choice(tooShorts).label
            return 'question'
        return None


class Quieten(State):
    """
    Too many people are currently talking. Attempt to return to a single
    speaker.
    """
    outcomes = ['success', 'select_other']
    input_data = ['name']
    output_data = ['name']

    def _execute(self, userdata):
        speaker = self.speakerStates.speakers[userdata.name]
        # Ask to be quiet with nao
        extraKey = [] if random.random() < EXTRA_KEYWORD_THRESHOLD else ["directed"]  # Maybe force directed
        self.req(keywords=["stop"] + extraKey, name=speaker.label, direction=speaker.azimuth)
        speaker.startTimeout(SINGLE_TIMEOUT)
        return 'select_other'


class SelectOtherAfterQuieten(State):

    outcomes = ['success']
    input_data = ['name']

    def _execute(self, userdata):
        chosenSpeaker = self.speakerStates.getLowestWeightedSpeaker()
        if chosenSpeaker is not None and chosenSpeaker.label != userdata.name:
            rospy.sleep(0.8)
            # Don't shut someone up and then select them again
            self.req(keywords=["start"], name=chosenSpeaker.label, direction=chosenSpeaker.azimuth)
            chosenSpeaker.startTimeout(SINGLE_TIMEOUT)
        return 'success'


class Question(State):
    outcomes = ['success']
    input_data = ['name']

    def _execute(self, userdata):
        chosenSpeaker = self.speakerStates.speakers[userdata.name]
        self.req(keywords=["start"], name=chosenSpeaker.label, direction=chosenSpeaker.azimuth)
        chosenSpeaker.startTimeout(SINGLE_TIMEOUT)
        return 'success'


class HappyNamed(State):
    """
    Show the previous action was successful.
    """

    outcomes = ['success']
    input_data = ['name']

    def _execute(self, userdata):
        rospy.sleep(2)
        if random.random() < SAY_THANK_YOU_THRESHOLD:
            return 'success'

        speaker = self.speakerStates.speakers[userdata.name]
        rospy.sleep(1)
        if not speaker.speaking:
            self.req(keywords=["thanks"], name=speaker.label, direction=speaker.azimuth)
        return 'success'


class GroupQuestion(State):

    outcomes = ['success', 'skipped']

    def _execute(self, userdata):
        if not self.client.allowGroup:
            return 'skipped'
        self.req(keywords=["start_anyone"], name="", direction=0.0)
        self.client.startTimeout(GROUP_TIMEOUT)
        return 'success'


class GroupQuieten(State):

    outcomes = ['success', 'skipped']

    def _execute(self, userdata):
        if not self.client.allowGroup:
            return 'skipped'
        self.req(keywords=["stop_anyone"], name="", direction=0.0)
        self.client.startTimeout(GROUP_TIMEOUT)
        return 'success'


class Happy(State):
    """
    Show the previous action was successful.
    """

    outcomes = ['success']

    def _execute(self, userdata):
        rospy.sleep(2)
        if random.random() < SAY_THANK_YOU_THRESHOLD:
            return 'success'

        if len(self.speakerStates.getActiveSpeakers()) == 0:
            self.req(keywords=["thanks"], name="", direction=0.0)
        return 'success'


class CloseTopic(State):
    """
    Wrap up the current topic.
    """

    outcomes = ['finished']

    def _execute(self, userdata):
        rospy.sleep(1)
        self.checkPreemption()
        self.req(keywords=["outro"], name="", direction=0.0)
        return 'finished'


def started(data):
    global running
    running = True


def main():
    rospy.init_node('mediatorbot_state_machine', log_level=rospy.DEBUG)

    speakerStates = SpeakerStates()
    try:
        actionclient = ActionClient()
    except ActionClientException as e:
        rospy.logerr(e)
        return
    rospy.Subscriber("start_recognition", StartRecognitionMsg, started)
    rospy.Subscriber("/added_user", AddedUser, speakerStates.addNewSpeaker)
    rospy.Subscriber("/speaker_change_state", MedBotSpeechStatus,
                     speakerStates.updateSpeaker)  # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end', 'preempted', 'errored'])

    with sm:
        # Add states to the container
        smach.StateMachine.add('Initialise', Initialise(speakerStates, actionclient),
                               transitions={'initialised': 'StartTopic',
                                            'preempted': 'preempted',
                                            'errored': 'errored'})
        smach.StateMachine.add('LookAtSpeaker', LookAtSpeaker(speakerStates, actionclient),
                               transitions={'looked': 'LookAtSpeaker',
                                            'preempted': 'preempted',
                                            'errored': 'errored',
                                            'finished': 'CloseTopic',
                                            'quieten': 'Quieten',
                                            'question': 'Question',
                                            'group_question': 'GroupQuestion',
                                            'group_quieten': 'GroupQuieten'},
                               remapping={'name': 'sm_name'})
        smach.StateMachine.add('StartTopic', StartTopic(speakerStates, actionclient),
                               transitions={'intro_complete': 'LookAtSpeaker',
                                            'preempted': 'preempted',
                                            'errored': 'errored'})
        smach.StateMachine.add('Quieten', Quieten(speakerStates, actionclient),
                               transitions={'success': 'HappyNamed',
                                            'select_other': 'SelectOtherAfterQuieten',
                                            'preempted': 'preempted',
                                            'errored': 'errored'},
                               remapping={'name': 'sm_name'})
        smach.StateMachine.add('HappyNamed', HappyNamed(speakerStates, actionclient),
                               transitions={'success': 'LookAtSpeaker',
                                            'preempted': 'preempted',
                                            'errored': 'errored'},
                               remapping={'name': 'sm_name'})
        smach.StateMachine.add('CloseTopic', CloseTopic(speakerStates, actionclient),
                               transitions={'finished': 'end',
                                            'preempted': 'preempted',
                                            'errored': 'errored'})
        smach.StateMachine.add('SelectOtherAfterQuieten', SelectOtherAfterQuieten(speakerStates, actionclient),
                               transitions={'success': 'LookAtSpeaker',
                                            'preempted': 'preempted',
                                            'errored': 'errored'},
                               remapping={'name': 'sm_name'})
        smach.StateMachine.add('Question', Question(speakerStates, actionclient),
                               transitions={'success': 'LookAtSpeaker',
                                            'preempted': 'preempted',
                                            'errored': 'errored'},
                               remapping={'name': 'sm_name'})
        smach.StateMachine.add('GroupQuestion', GroupQuestion(speakerStates, actionclient),
                               transitions={'success': 'LookAtSpeaker',
                                            'skipped': 'LookAtSpeaker',
                                            'preempted': 'preempted',
                                            'errored': 'errored'})
        smach.StateMachine.add('GroupQuieten', GroupQuieten(speakerStates, actionclient),
                               transitions={'success': 'Happy',
                                            'skipped': 'LookAtSpeaker',
                                            'preempted': 'preempted',
                                            'errored': 'errored'})
        smach.StateMachine.add('Happy', Happy(speakerStates, actionclient),
                               transitions={'success': 'LookAtSpeaker',
                                            'preempted': 'preempted',
                                            'errored': 'errored'})

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
