import math
import rospy
import time

import stopLeft
import stopRight
import pointRight
import pointLeft
import youRight
import youLeft


def lookAtDirection(movementProxy, direction):
    movementProxy.setAngles("HeadYaw", direction, 0.3)


class Action(object):
    actionKeyWord = ""
    keywords = set()

    def __init__(self, movementProxy, textToSpeechProxy, behaviourManagerProxy, keywords=None):
        keywords = set() if keywords is None else set(keywords)

        self.movementProxy = movementProxy
        self.textToSpeechProxy = textToSpeechProxy
        self.behaviourManagerProxy = behaviourManagerProxy
        self.keywords = set(self.keywords) | keywords

    def run(self, *args, **kwargs):
        raise NotImplementedError

    def __str__(self):
        return self.__class__.__name__


class ResponseAction(Action):
    def run(self, name):
        rospy.loginfo("Response: {:s} to {}".format(self, name))
        self._response(name)

    def _response(self, name):
        raise NotImplementedError


class MovementAction(Action):
    def run(self, direction):
        rospy.loginfo("Movement: {:s} to {}".format(self, direction))
        self._movement(direction)

    def _movement(self, direction):
        raise NotImplementedError


class StopSomeoneElseResponse(ResponseAction):
    keywords = {"stop", "select_other"}

    def _response(self, name):
        self.behaviourManagerProxy.runBehavior("reponses-c397b4/Someone_else")


class StopRaiseArmMotion(MovementAction):
    keywords = {"stop"}

    def _movement(self, direction):
        direction = math.radians(direction)
        action = stopRight.StopRight(direction, self.movementProxy) if direction < 0 else stopLeft.StopLeft(direction,
                                                                                                            self.movementProxy)
        lookAtDirection(self.movementProxy, direction)
        action.run()


class StopShutUpResponse(ResponseAction):
    keywords = {"stop", "angry"}

    def _response(self, name):
        self.textToSpeechProxy.say("{}, shut up".format(name))


class StopThanksResponse(ResponseAction):
    keywords = {"stop", "polite"}

    def _response(self, name):
        self.textToSpeechProxy.say("Thank you for your contribution {}".format(name))


class StartAnyoneMovement(MovementAction):
    keywords = {"anyone", "select_other", "polite"}

    def _movement(self, direction):
        direction = math.radians(direction)
        lookAtDirection(self.movementProxy, direction)
        self.behaviourManagerProxy.runBehavior("actions-67d9a5/Anyone")


class StartPointMovement(MovementAction):
    keywords = {"start", "select_other", "directed"}

    def _movement(self, direction):
        direction = math.radians(direction)
        action = pointRight.PointRight(direction, self.movementProxy) if direction < 0 else pointLeft.PointLeft(
            direction, self.movementProxy)
        lookAtDirection(self.movementProxy, direction)
        action.run()


class StartAnyThoughtsResponse(ResponseAction):
    keywords = {"start", "polite", "directed"}

    def _response(self, name):
        self.textToSpeechProxy.say("Do you have any thoughts on this matter, {}".format(name))


class ShareThoughtsResponse1(ResponseAction):
    keywords = {"start", "directed"}

    def _response(self, name):
        self.behaviourManagerProxy.runBehavior("reponses-c397b4/Share")


class ShareThoughtsResponse2(ResponseAction):
    keywords = {"anyone", "thoughts"}

    def _response(self, name):
        self.textToSpeechProxy.say("Does anyone have thoughts on this matter?")


class PointPalmUpAction(MovementAction):
    keywords = {"polite", "directed"}

    def _movement(self, direction):
        direction = math.radians(direction)
        action = youRight.YouRight(direction, self.movementProxy) if direction < 0 else youLeft.YouLeft(
            direction, self.movementProxy)
        lookAtDirection(self.movementProxy, direction)
        action.run()


class YouTurnResponse(ResponseAction):
    keywords = {"start", "directed"}

    def _response(self, name):
        self.behaviourManagerProxy.runBehavior("reponses-c397b4/Your_turn")


class YouViewResponse(ResponseAction):
    keywords = {"start", "polite", "directed"}

    def _response(self, name):
        self.behaviourManagerProxy.runBehavior("reponses-c397b4/Your_views")


class StopDidISAskYouResponse(ResponseAction):
    keywords = {"stop", "angry", "directed"}

    def _response(self, name):
        self.textToSpeechProxy.say("Did I ask for your input, {}".format(name))


class StopTooLoudMovement(MovementAction):
    keywords = {"stop", "loud"}

    def _movement(self, direction):
        self.behaviourManagerProxy.runBehavior("actions-67d9a5/Loud")


class StopQuietMovement(MovementAction):
    keywords = {"stop", "loud"}

    def _movement(self, direction):
        self.behaviourManagerProxy.runBehavior("actions-67d9a5/Quiet")


class StopShhMovement(MovementAction):
    keywords = {"stop", "loud", "polite"}

    def _movement(self, direction):
        self.behaviourManagerProxy.runBehavior("actions-67d9a5/Ssh")


class StopQuietEmptyResponse(ResponseAction):
    keywords = {"loud", "stop"}

    def _response(self, name):
        pass


class StopQuietResponse(ResponseAction):
    keywords = {"stop", "loud"}

    def _response(self, name):
        self.behaviourManagerProxy.runBehavior("reponses-c397b4/Speak_quiet")


class StopShhResponse(ResponseAction):
    keywords = {"stop", "loud"}

    def _response(self, name):
        self.textToSpeechProxy.say("Shush")


class StopMultipleMovement(MovementAction):
    keywords = {"stop", "multiple"}

    def _movement(self, direction):
        self.behaviourManagerProxy.runBehavior("actions-67d9a5/Stop")


class StopMultipleResponse(ResponseAction):
    keywords = {"multiple"}

    def _response(self, name):
        self.behaviourManagerProxy.runBehavior("reponses-c397b4/One_speaker")


class StopMultipleResponse2(ResponseAction):
    keywords = {"polite", "multiple"}

    def _response(self, name):
        self.textToSpeechProxy.say("Please, one person at a time")

class LookAtSpeaker(MovementAction):
    keywords = {"look", "natural"}

    def _movement(self, direction):
        direction = math.radians(direction)
        self.movementProxy.setAngles("HeadYaw", direction, 0.3)
        time.sleep(1)
