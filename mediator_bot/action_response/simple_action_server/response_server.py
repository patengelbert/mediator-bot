#! /usr/bin/env python
import rospy
import actionlib
import random

from naoqi import ALProxy

from action_response.msg import responseAction

ALLOW_ANGRY_ACTIONS = False

from actions import (
    MovementAction,
    ResponseAction,
    StopSomeoneElseResponse,
    StopRaiseArmMotion,
    StopThanksResponse,
    StartAnyoneMovement,
    StartPointMovement,
    StartAnyThoughtsResponse,
    ShareThoughtsResponse1,
    ShareThoughtsResponse2,
    PointPalmUpAction,
    YouTurnResponse,
    YouViewResponse,
    StopTooLoudMovement,
    StopQuietMovement,
    StopShhMovement,
    StopQuietEmptyResponse,
    StopQuietResponse,
    StopShhResponse,
    StopMultipleResponse,
    StopMultipleMovement,
    StopMultipleResponse2,
    LookAtSpeaker,
    StopDidISAskYouResponse,
    StopMultipleResponseNamed,
    IntroOutroMovement,
    IntroResponse,
    OutroResponse,
    NearlyDoneResponse,
    ThankYouResponse,
    ThankYouResponseNamed,
)

# NAO IP address and port

robotIP = "169.254.44.123"
robotPort = 9559

actionsToAdd = {
    StopSomeoneElseResponse,
    StopRaiseArmMotion,
    StopThanksResponse,
    StartAnyoneMovement,
    StartPointMovement,
    StartAnyThoughtsResponse,
    ShareThoughtsResponse1,
    ShareThoughtsResponse2,
    PointPalmUpAction,
    YouTurnResponse,
    YouViewResponse,
    StopTooLoudMovement,
    StopQuietMovement,
    StopShhMovement,
    StopQuietEmptyResponse,
    StopQuietResponse,
    StopShhResponse,
    StopMultipleMovement,
    StopMultipleResponse,
    StopMultipleResponse2,
    LookAtSpeaker,
    StopDidISAskYouResponse,
    StopMultipleResponseNamed,
    IntroOutroMovement,
    IntroResponse,
    OutroResponse,
    NearlyDoneResponse,
    ThankYouResponse,
    ThankYouResponseNamed,
}

if not ALLOW_ANGRY_ACTIONS:
    actionsToAdd = set([a for a in actionsToAdd if "angry" not in a.keywords])


def changeName(name):
    if name.lower() == "yiannis":
        return "yannis"
    return name


class ActionLibraryError(Exception):
    def __str__(self):
        return "Error in action library"


class ActionLibrary(object):
    movementActions = []
    responseActions = []

    def addAction(self, action):
        if isinstance(action, MovementAction):
            self.movementActions.append(action)
        elif isinstance(action, ResponseAction):
            self.responseActions.append(action)
        else:
            rospy.logerr("Unknown type {} of action".format(type(action)))
            raise ActionLibraryError()

    def _getAction(self, actions, keywords):
        keywords = set(keywords)
        rospy.loginfo("Searching for actions for {}".format(','.join(str(k) for k in keywords)))

        if len(actions) == 0:
            rospy.logwarn("No actions have been registered of this type")
            return

        def getTotalOccurences(listA, listB):
            t = 0
            for i in listA:
                t += 1 if i in listB else 0
            return t

        possibleActions = sorted(map(lambda x: (getTotalOccurences(x.keywords, keywords), x), actions),
                                 key=lambda x: x[0], reverse=True)

        rospy.logdebug(','.join(["{}:{}".format(x, w) for w, x in possibleActions]))

        topWeight = possibleActions[0][0]

        if topWeight == 0:
            # We do not want completely unrelated actions from occurring
            rospy.logwarn("Found no applicable actions")
            return None

        possibleActions = [action for weight, action in possibleActions if weight == topWeight]
        rospy.logdebug("Found {}/{} possible actions".format(len(possibleActions), len(actions)))

        if len(possibleActions) == 1:
            return possibleActions[0]

        return random.choice(possibleActions)

    def getMovementAction(self, keywords):
        return self._getAction(self.movementActions, keywords)

    def getResponseAction(self, keywords):
        return self._getAction(self.responseActions, keywords)


actionLibrary = ActionLibrary()


class ResponseServer:
    def __init__(self, debugLevel=rospy.INFO):
        rospy.init_node('response_server', log_level=debugLevel)
        rospy.on_shutdown(self.shutdown)
        # Initialise and start the action server
        self.server = actionlib.SimpleActionServer('response', responseAction, self.execute, False)

        try:
            rospy.loginfo("Loading proxies")
            # Initialise the behaviour manager for the NAO
            self.tts = ALProxy("ALTextToSpeech", robotIP, robotPort)
            self.tts.setParameter("pitchShift", 1.0)

            self.bm = ALProxy("ALBehaviorManager", robotIP, robotPort)

            self.mp = ALProxy("ALMotion", robotIP, 9559)
            self.mp.wakeUp()

        except Exception as e:
            rospy.logerr("Could not create proxy")
            rospy.logerr("Error was: {}".format(e))
            raise

        self.addActions()

        self.bm.startBehavior("actions-67d9a5/Breathe")

        self.server.start()

    def addActions(self):
        for action in actionsToAdd:
            actionLibrary.addAction(action(self.mp, self.tts, self.bm))

    def shutdown(self):
        if hasattr(self, 'bm') and self.bm is not None:
            self.bm.stopAllBehaviors()
        if hasattr(self, 'mp') and self.mp is not None:
            self.mp.rest()

    def execute(self, goal):
        self.bm.stopAllBehaviors()
        kw = goal.keywords
        m = actionLibrary.getMovementAction(kw)
        r = actionLibrary.getResponseAction(kw)
        if m is not None:
            dir = goal.direction if goal.direction is not None else 0.0
            m.run(max(min(dir, 90.0), -90.0))
        if r is not None:
            name = goal.name if goal.name is not None else "You"
            r.run(changeName(name))

        self.bm.runBehavior("actions-67d9a5/Return")
        self.bm.startBehavior("actions-67d9a5/Breathe")

        self.server.set_succeeded()

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    server = ResponseServer(debugLevel=rospy.DEBUG)
    server.run()
