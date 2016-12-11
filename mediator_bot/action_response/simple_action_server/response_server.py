#! /usr/bin/env python

import roslib
roslib.load_manifest('action_response')
import rospy
import actionlib
import random
from naoqi import ALProxy

from action_response.msg import responseAction

# NAO IP address and port
robotIP = "169.254.44.123"
robotPort = 9559
# Lists of actions in the keyword groups
actionStopList = [1]
actionStartList = [1, 2, 3]
actionLoudList = [1, 2, 3]
actionMultipleList = [1]
actionNaturalList = [0]
# Lists of responses in the keyword groups
responseStopList = [1]
responseStartList = [1, 2, 3]
responseLoudList = [1]
responseMultipleList = [1]


class ResponseServer:
  def __init__(self):
    # Initialise and start the action server
    self.server = actionlib.SimpleActionServer('response', responseAction, self.execute, False)
    self.server.start()
    # Initialise the monitor lists for sudo-random action and response selection for each keyword group
    self.stopMonitor = actionStopList[:]
    self.startMonitor = actionStartList[:]
    self.loudMonitor = actionLoudList[:]
    self.multipleMonitor = actionMultipleList[:]
    self.naturalMonitor = actionNaturalList[:]
    self.resStopMonitor = responseStopList[:]
    self.resStartMonitor = responseStartList[:]
    self.resLoudMonitor = responseLoudList[:]
    self.resMultipleMonitor = responseMultipleList[:]

    # Initialise the behaviour manager for the NAO
    #self.tts = ALProxy("ALTextToSpeech", robotIP, robotPort)
    self.bm = ALProxy("ALBehaviourManager", robotIP, robotPort)
    #self.tts.say("Starting")

  def execute(self, goal):
    
    # If stop is commanded (name and direction required)
    if goal.action == "stop":
        # Select action to run
        if not self.stopMonitor:
            self.stopMonitor = actionStopList[:]
        chosen = random.choice(self.stopMonitor)
        if chosen is 1:
            print "stop ", goal.name, ", ", goal.direction 
            #self.bm.runBehavior("actions-67d9a5/Stop")
        self.stopMonitor.remove(chosen)
        # Select response to run
        if not self.resStopMonitor:
            self.resStopMonitor = responseStopList[:]
        chosen = random.choice(self.resStopMonitor)
        if chosen is 1:
            print "someone else ", goal.name, ", ", goal.direction
            #self.bm.runBehavior("reponses-c397b4/Someone_else")
        self.resStopMonitor.remove(chosen)
        #self.bm.runBehavior("actions-67d9a5/Return")

    # If start is commanded (name and direction required)
    if goal.action == "start":
        # Select action to run
        if not self.startMonitor:
            self.startMonitor = actionStartList[:]
        chosen = random.choice(self.startMonitor)
        if chosen is 1:
            print "anyone ", goal.name, ", ", goal.direction
            #self.bm.runBehavior("actions-67d9a5/Anyone")
        elif chosen is 2:
            print "point ", goal.name, ", ", goal.direction
            #self.bm.runBehavior("actions-67d9a5/Point")
	elif chosen is 3:
            print "you ", goal.name, ", ", goal.direction
            #self.bm.runBehavior("actions-67d9a5/You")
        self.startMonitor.remove(chosen)
        # Select response to run
        if not self.resStartMonitor:
            self.resStartMonitor = responseStartList[:]
        chosen = random.choice(self.resStartMonitor)
        if chosen is 1:
            print "share ", goal.name, ", ", goal.direction
            #self.bm.runBehavior("reponses-c397b4/Share")
        if chosen is 2:
            print "your turn ", goal.name, ", ", goal.direction
            #self.bm.runBehavior("reponses-c397b4/Your_turn")
        if chosen is 3:
            print "your views ", goal.name, ", ", goal.direction
            #self.bm.runBehavior("reponses-c397b4/Your_views")
        self.resStartMonitor.remove(chosen)
        #self.bm.runBehavior("actions-67d9a5/Return")

    # If loud is commanded
    if goal.action == "loud":
        # Select action to run
        if not self.loudMonitor:
            self.loudMonitor = actionLoudList[:]
        chosen = random.choice(self.loudMonitor)
        if chosen is 1:
            print "loud"
            #self.bm.runBehavior("actions-67d9a5/Loud")
        if chosen is 2:
            print "quiet"
            #self.bm.runBehavior("actions-67d9a5/Quiet")
        if chosen is 3:
            print "ssh"
            #self.bm.runBehavior("actions-67d9a5/Ssh")
        self.loudMonitor.remove(chosen)
        # Select response to run
        if not self.resLoudMonitor:
            self.resLoudMonitor = responseLoudList[:]
        chosen = random.choice(self.resLoudMonitor)
        if chosen is 1:
            print "speak quiet"
            #self.bm.runBehavior("reponses-c397b4/Speak_quiet")
        self.resLoudMonitor.remove(chosen)
        #self.bm.runBehavior("actions-67d9a5/Return")

    # If multiple is commmanded
    if goal.action == "multiple":
        # Selct action to run
	if not self.multipleMonitor:
            self.multipleMonitor = actionMultipleList[:]
        chosen = random.choice(self.multipleMonitor)
        if chosen is 1:
            print "stop_multiple"
            #self.bm.runBehavior("actions-67d9a5/Stop")
        self.multipleMonitor.remove(chosen)
        # Select response to run
        if not self.resMultipleMonitor:
            self.resMultipleMonitor = responseMultipleList[:]
        chosen = random.choice(self.resMultipleMonitor)
        if chosen is 1:
            print "one speaker"
            #self.bm.runBehavior("reponses-c397b4/One_speaker")
        self.resMultipleMonitor.remove(chosen)
        #self.bm.runBehavior("actions-67d9a5/Return")

    # If natural behaviour is commanded
    if goal.action == "natural":
	print "Act natural at %d" % goal.direction
        #self.bm.runBehavior("Return")

    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('response_server')
  server = ResponseServer()
  rospy.spin()
