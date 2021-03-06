#! /usr/bin/env python

class PointRight:
    def __init__(self, angle, movementProxy):
        self.names = list()
        self.times = list()
        self.keys = list()

        self.movementProxy = movementProxy

        self.names.append("LElbowRoll")
        self.times.append([0.04])
        self.keys.append([[-0.570283, [3, -0.0133333, 0], [3, 0, 0]]])

        self.names.append("LElbowYaw")
        self.times.append([0.04])
        self.keys.append([[-1.2094, [3, -0.0133333, 0], [3, 0, 0]]])

        self.names.append("LHand")
        self.times.append([0.04])
        self.keys.append([[0.52316, [3, -0.0133333, 0], [3, 0, 0]]])

        self.names.append("LShoulderPitch")
        self.times.append([0.04])
        self.keys.append([[1.54323, [3, -0.0133333, 0], [3, 0, 0]]])

        self.names.append("LShoulderRoll")
        self.times.append([0.04])
        self.keys.append([[0.17584, [3, -0.0133333, 0], [3, 0, 0]]])

        self.names.append("LWristYaw")
        self.times.append([0.04])
        self.keys.append([[0.09314, [3, -0.0133333, 0], [3, 0, 0]]])

        self.names.append("RElbowRoll")
        self.times.append([0.04, 0.48, 1.4])
        self.keys.append(
            [[0.413896, [3, -0.0133333, 0], [3, 0.146667, 0]], [0.0959151, [3, -0.146667, 0], [3, 0.306667, 0]],
             [0.0959151, [3, -0.306667, 0], [3, 0, 0]]])

        self.names.append("RElbowYaw")
        self.times.append([0.04, 0.48, 1.4])
        self.keys.append(
            [[1.18213, [3, -0.0133333, 0], [3, 0.146667, 0]], [1.11953, [3, -0.146667, 0], [3, 0.306667, 0]],
             [1.11953, [3, -0.306667, 0], [3, 0, 0]]])

        self.names.append("RHand")
        self.times.append([0.04, 0.48, 1.4])
        self.keys.append([[0.605445, [3, -0.0133333, 0], [3, 0.146667, 0]], [1, [3, -0.146667, 0], [3, 0.306667, 0]],
                          [1, [3, -0.306667, 0], [3, 0, 0]]])

        self.names.append("RShoulderPitch")
        self.times.append([0.04, 0.48, 1.4])
        self.keys.append(
            [[1.29516, [3, -0.0133333, 0], [3, 0.146667, 0]], [0.0497333, [3, -0.146667, 0], [3, 0.306667, 0]],
             [0.0497333, [3, -0.306667, 0], [3, 0, 0]]])

        self.names.append("RShoulderRoll")
        self.times.append([0.04, 0.48, 1.4])

        self.keys.append([[-0.133997, [3, -0.0133333, 0], [3, 0.146667, 0]], [angle, [3, -0.146667, 0], [3, 0.306667, 0]],
                          [angle, [3, -0.306667, 0], [3, 0, 0]]])

        self.names.append("RWristYaw")
        self.times.append([0.04, 0.48, 1.4])
        self.keys.append(
            [[0.100796, [3, -0.0133333, 0], [3, 0.146667, 0]], [-0.292563, [3, -0.146667, 0], [3, 0.306667, 0]],
             [-0.292563, [3, -0.306667, 0], [3, 0, 0]]])

    def run(self):
        self.movementProxy.angleInterpolationBezier(self.names, self.times, self.keys)

