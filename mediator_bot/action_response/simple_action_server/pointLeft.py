#! /usr/bin/env python

import rospy
from naoqi import ALProxy

robotIP = "127.0.0.1"
robotPort = 57373


class PointLeft:
    def __init__(self, angle):
        self.names = list()
        self.times = list()
        self.keys = list()

        self.names.append("LElbowRoll")
        self.times.append([0.04, 0.48, 1.28])
        self.keys.append(
            [[-0.413896, [3, -0.0133333, 0], [3, 0.146667, 0]], [-0.0959151, [3, -0.146667, 0], [3, 0.266667, 0]],
             [-0.0959151, [3, -0.266667, 0], [3, 0, 0]]])

        self.names.append("LElbowYaw")
        self.times.append([0.04, 0.48, 1.28])
        self.keys.append(
            [[-1.18213, [3, -0.0133333, 0], [3, 0.146667, 0]], [-1.11953, [3, -0.146667, 0], [3, 0.266667, 0]],
             [-1.11953, [3, -0.266667, 0], [3, 0, 0]]])

        self.names.append("LHand")
        self.times.append([0.04, 0.48, 1.28])
        self.keys.append([[0.605445, [3, -0.0133333, 0], [3, 0.146667, 0]], [1, [3, -0.146667, 0], [3, 0.266667, 0]],
                          [1, [3, -0.266667, 0], [3, 0, 0]]])

        self.names.append("LShoulderPitch")
        self.times.append([0.04, 0.48, 1.28])
        self.keys.append(
            [[1.29516, [3, -0.0133333, 0], [3, 0.146667, 0]], [0.0497333, [3, -0.146667, 0], [3, 0.266667, 0]],
             [0.0497333, [3, -0.266667, 0], [3, 0, 0]]])

        self.names.append("LShoulderRoll")
        self.times.append([0.04, 0.48, 1.28])

        self.keys.append([[0.133997, [3, -0.0133333, 0], [3, 0.146667, 0]], [angle, [3, -0.146667, 0], [3, 0.266667, 0]],
                          [angle, [3, -0.266667, 0], [3, 0, 0]]])

        self.names.append("LWristYaw")
        self.times.append([0.04, 0.48, 1.28])
        self.keys.append(
            [[-0.100796, [3, -0.0133333, 0], [3, 0.146667, 0]], [0.292563, [3, -0.146667, 0], [3, 0.266667, 0]],
             [0.292563, [3, -0.266667, 0], [3, 0, 0]]])

        self.names.append("RElbowRoll")
        self.times.append([0.04, 0.48, 1.28])
        self.keys.append(
            [[0.413896, [3, -0.0133333, 0], [3, 0.146667, 0]], [0.418879, [3, -0.146667, 0], [3, 0.266667, 0]],
             [0.418879, [3, -0.266667, 0], [3, 0, 0]]])

        self.names.append("RElbowYaw")
        self.times.append([0.04, 0.48, 1.28])
        self.keys.append(
            [[1.18213, [3, -0.0133333, 0], [3, 0.146667, 0]], [1.19381, [3, -0.146667, 0], [3, 0.266667, 0]],
             [1.19381, [3, -0.266667, 0], [3, 0, 0]]])

        self.names.append("RHand")
        self.times.append([0.04, 0.48, 1.28])
        self.keys.append([[0.605445, [3, -0.0133333, 0], [3, 0.146667, 0]], [0.3, [3, -0.146667, 0], [3, 0.266667, 0]],
                          [0.3, [3, -0.266667, 0], [3, 0, 0]]])

        self.names.append("RShoulderPitch")
        self.times.append([0.04, 0.48, 1.28])
        self.keys.append([[1.29516, [3, -0.0133333, 0], [3, 0.146667, 0]],
                          [1.47131, [3, -0.146667, -7.32369e-08], [3, 0.266667, 1.33158e-07]],
                          [1.47131, [3, -0.266667, 0], [3, 0, 0]]])

        self.names.append("RShoulderRoll")
        self.times.append([0.04, 0.48, 1.28])
        self.keys.append(
            [[-0.133997, [3, -0.0133333, 0], [3, 0.146667, 0]], [-0.185005, [3, -0.146667, 0], [3, 0.266667, 0]],
             [-0.185005, [3, -0.266667, 0], [3, 0, 0]]])

        self.names.append("RWristYaw")
        self.times.append([0.04, 0.48, 1.28])
        self.keys.append(
            [[0.100796, [3, -0.0133333, 0], [3, 0.146667, 0]], [0.0994838, [3, -0.146667, 0], [3, 0.266667, 0]],
             [0.0994838, [3, -0.266667, 0], [3, 0, 0]]])

    def run(self, IP, Port):

        try:
            motion = ALProxy("ALMotion", IP, Port)
            motion.angleInterpolationBezier(self.names, self.times, self.keys)
        except Exception as err:
            rospy.logerr(err)
            raise


if __name__ == "__main__":
    pl = PointLeft(45)
    pl.run(robotIP, robotPort)
