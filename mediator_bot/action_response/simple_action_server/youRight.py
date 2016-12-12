#! /usr/bin/env python

from naoqi import ALProxy
import math

robotIP = "127.0.0.1"
robotPort = 57373

class YouRight:
    def __init__(self,angle):
        self.names = list()
        self.times = list()
        self.keys = list()

        self.names.append("LElbowRoll")
        self.times.append([0.04])
        self.keys.append([[-0.360836, [3, -0.0133333, 0], [3, 0, 0]]])

        self.names.append("LElbowYaw")
        self.times.append([0.04])
        self.keys.append([[-1.19997, [3, -0.0133333, 0], [3, 0, 0]]])
        
        self.names.append("LHand")
        self.times.append([0.04])
        self.keys.append([[0.799694, [3, -0.0133333, 0], [3, 0, 0]]])

        self.names.append("LShoulderPitch")
        self.times.append([0.04])
        self.keys.append([[1.57586, [3, -0.0133333, 0], [3, 0, 0]]])

        self.names.append("LShoulderRoll")
        self.times.append([0.04])
        self.keys.append([[0.209645, [3, -0.0133333, 0], [3, 0, 0]]])

        self.names.append("LWristYaw")
        self.times.append([0.04])
        self.keys.append([[-0.126919, [3, -0.0133333, 0], [3, 0, 0]]])

        self.names.append("RElbowRoll")
        self.times.append([0.04, 0.6, 1.6])
        self.keys.append([[0.292386, [3, -0.0133333, 0], [3, 0.186667, 0]], [0.213496, [3, -0.186667, 0], [3, 0.333333, 0]], [0.213496, [3, -0.333333, 0], [3, 0, 0]]])

        self.names.append("RElbowYaw")
        self.times.append([0.04, 0.6, 1.6])
        self.keys.append([[1.1802, [3, -0.0133333, 0], [3, 0.186667, 0]], [1.47932, [3, -0.186667, 0], [3, 0.333333, 0]], [1.47932, [3, -0.333333, 0], [3, 0, 0]]])

        self.names.append("RHand")
        self.times.append([0.04, 0.6, 1.6])
        self.keys.append([[0.816771, [3, -0.0133333, 0], [3, 0.186667, 0]], [1, [3, -0.186667, 0], [3, 0.333333, 0]], [1, [3, -0.333333, 0], [3, 0, 0]]])

        self.names.append("RShoulderPitch")
        self.times.append([0.04, 0.6, 1.6])
        self.keys.append([[1.30951, [3, -0.0133333, 0], [3, 0.186667, 0]], [0.199015, [3, -0.186667, 0], [3, 0.333333, 0]], [0.199015, [3, -0.333333, 0], [3, 0, 0]]])

        self.names.append("RShoulderRoll")
        self.times.append([0.04, 0.6, 1.6])
        rad = math.radians(angle)
	if(rad > 1.7):
            rad = 0.02
        self.keys.append([[-0.128319, [3, -0.0133333, 0], [3, 0.186667, 0]], [rad, [3, -0.186667, 0], [3, 0.333333, 0]], [rad, [3, -0.333333, 0], [3, 0, 0]]])

        self.names.append("RWristYaw")
        self.times.append([0.04, 0.6, 1.6])
        self.keys.append([[0.100401, [3, -0.0133333, 0], [3, 0.186667, 0]], [1.53059, [3, -0.186667, 0], [3, 0.333333, 0]], [1.53059, [3, -0.333333, 0], [3, 0, 0]]])

    def run(self):

        try:
            motion = ALProxy("ALMotion", robotIP, robotPort)
            motion.angleInterpolationBezier(self.names, self.times, self.keys)
        except BaseException, err:
            print err

if __name__=="__main__":

    yr = YouRight(-45)
    yr.run()
