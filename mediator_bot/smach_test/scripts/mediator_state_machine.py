#! /usr/bin/env python
import threading

import rospy
import smach
import smach_ros
from mediator_bot_msgs.msg import MedBotSpeechStatus
from speech_speaker_recognition.msg import AddedUser


# TODO import other timing messages

THRESHOLD = 1


class SpeakerStates:
    """
    Container for current speaker states
    """
    init = False
    speakers = []
    weight = {}
    status = {}
    speaking = {}

    def checkSpeakerLevels(self):
        # check if anyones spoken too much
        # return id and level
        return [key for key, v in self.weight.iteritems() if v > THRESHOLD]

    def checkSpeakers(self):
        # Check how many people are currently speaking
        # return speaker ids
        return [key for key, v in self.speaking.itervalues() if v is True]

speakerStates = SpeakerStates()


## States ##

# define state Initialise
class Initialise(smach.State):
    """
    Initialise the module and wait to start.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['initialised', 'preempted'])

    def execute(self, userdata):
        # get names from topic, added user
        rospy.loginfo('Executing state Initialise')
        rospy.loginfo('Waiting for registered users...')
        while not SpeakerStates.speakers:  # wait while there are no registered
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1)
        rospy.loginfo('Users registered')
        raw_input("Press Enter to continue...")
        return 'initialised'

    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        smach.State.request_preempt(self)
        rospy.logwarn("Preempted!")


# define state Start
class StartTopic(smach.State):
    """
    Start the current topic
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['intro_complete'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Start')
        rospy.sleep(1)
        self.introSequence()
        return 'intro_complete'

    def introSequence(self):
        rospy.loginfo('*Intro Sequence*')


# define state Mediate
class Mediate(smach.State):
    """
    Main state mediating the conversation. Constantly checking the state of
    the conversation in order to give a appropriate response.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['timeup', 'control_conv', 'not_speaking'])
        self.timeup = False
        self.init = False

    def execute(self, userdata):
        rospy.loginfo('Executing state Mediate')
        rospy.sleep(1)
        if self.init is False:
            self.initTimer()
        while not rospy.is_shutdown():
            # check all speakers
            # self.checkSpeakLevels();
            # self.checkSpeakers();
            if self.timeup is True:
                # time run out
                return 'timeup'
            elif False:
                # someones talking too much
                return 'not_speaking'
            elif True:
                # too many speakers
                return 'control_conv'

    def initTimer(self):
        rospy.Timer(rospy.Duration(30), self.callback)
        self.init = True

    def callback(self, event):
        rospy.loginfo('Time is up: ' + str(event.current_real))
        self.timeup = True


# define state Quieten
class Quieten(smach.State):
    """
    Too many people are currently talking. Attempt to return to a single
    speaker.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Quieten')
        # Ask to be quiet with nao
        rospy.sleep(3)
        # Check if there is only 1 speaker
        if True:
            return 'failed'
        else:
            return 'success'
            # If failed try again


# define state Happy
class Happy(smach.State):
    """
    Show the previous action was successful.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Happy')
        rospy.sleep(1)
        return 'success'


# define state AskQuestion
class AskQuestion(smach.State):
    """
    Ask least talkative person a question if someone talking too much.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('Executing state AskQuestion')
        # direct question at person
        rospy.sleep(1)
        return 'success'


# define state Start
class CloseTopic(smach.State):
    """
    Wrap up the current topic.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['finished', 'next_topic'])

    def execute(self, userdata):
        rospy.loginfo('Executing state CloseTopic')
        rospy.sleep(1)
        return 'finished'


## end of states ##

## Callback funcs
def callbackSpeakerState(data):
    speakerStates.weight[data.name] = data.weight
    speakerStates.status[data.name] = data.status
    speakerStates.speaking[data.name] = data.speaking


def callbackNewSpeaker(data):
    # register speakers
    speakerStates.speakers.append(data.name)
    speakerStates.weight[data.name] = 0.0
    speakerStates.status[data.name] = 0
    speakerStates.speaking[data.name] = False


# def callbackLoud(): # too loud
# Compute average power of the frames and make sure it doesnt exceed a threshold

## Status checking functions ##


def main():
    rospy.init_node('mediatorbot_state_machine', log_level=rospy.DEBUG)

    rospy.Subscriber("/speaker", AddedUser, callbackNewSpeaker)
    rospy.Subscriber("/speaker_change_state", MedBotSpeechStatus, callbackSpeakerState)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end', 'preempted'])

    # sis = smach_ros.IntrospectionServer('server_name', sm, '/mediatorbot')
    # sis.start()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Initialise', Initialise(),
                               transitions={'initialised': 'StartTopic',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('StartTopic', StartTopic(),
                               transitions={'intro_complete': 'Mediate'})
        smach.StateMachine.add('Mediate', Mediate(),
                               transitions={'timeup': 'CloseTopic', 'control_conv': 'Quieten',
                                            'not_speaking': 'AskQuestion'})
        smach.StateMachine.add('Quieten', Quieten(),
                               transitions={'success': 'Happy', 'failed': 'Mediate'})
        smach.StateMachine.add('Happy', Happy(),
                               transitions={'success': 'Mediate'})
        smach.StateMachine.add('AskQuestion', AskQuestion(),
                               transitions={'success': 'Mediate'})
        smach.StateMachine.add('CloseTopic', CloseTopic(),
                               transitions={'finished': 'end', 'next_topic': 'StartTopic'})

    # Execute SMACH plan
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.daemon = True
    smach_thread.start()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sm.request_preempt()

    rospy.logdebug("Waiting for smach to shut down")

    # Block until everything is preempted
    # (you could do something more complicated to get the execution outcome if you want it)
    smach_thread.join()
    # sis.stop()


if __name__ == '__main__':
    main()
