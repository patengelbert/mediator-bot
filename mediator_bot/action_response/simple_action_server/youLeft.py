#! /usr/bin/env python

class YouLeft:
    def __init__(self, angle, movementProxy):
        self.names = list()
        self.times = list()
        self.keys = list()

        self.movementProxy = movementProxy

        self.names.append("LElbowRoll")
        self.times.append([0.04, 0.6, 1.56])
        self.keys.append(
            [[-0.292386, [3, -0.0133333, 0], [3, 0.186667, 0]], [-0.213496, [3, -0.186667, 0], [3, 0.32, 0]],
             [-0.213496, [3, -0.32, 0], [3, 0, 0]]])

        self.names.append("LElbowYaw")
        self.times.append([0.04, 0.6, 1.56])
        self.keys.append([[-1.1802, [3, -0.0133333, 0], [3, 0.186667, 0]], [-1.47932, [3, -0.186667, 0], [3, 0.32, 0]],
                          [-1.47932, [3, -0.32, 0], [3, 0, 0]]])

        self.names.append("LHand")
        self.times.append([0.04, 0.6, 1.56])
        self.keys.append([[0.816771, [3, -0.0133333, 0], [3, 0.186667, 0]], [1, [3, -0.186667, 0], [3, 0.32, 0]],
                          [1, [3, -0.32, 0], [3, 0, 0]]])

        self.names.append("LShoulderPitch")
        self.times.append([0.04, 0.6, 1.56])
        self.keys.append([[1.30951, [3, -0.0133333, 0], [3, 0.186667, 0]], [0.199015, [3, -0.186667, 0], [3, 0.32, 0]],
                          [0.199015, [3, -0.32, 0], [3, 0, 0]]])

        self.names.append("LShoulderRoll")
        self.times.append([0.04, 0.6, 1.56])

        self.keys.append([[0.128319, [3, -0.0133333, 0], [3, 0.186667, 0]], [angle, [3, -0.186667, 0], [3, 0.32, 0]],
                          [angle, [3, -0.32, 0], [3, 0, 0]]])

        self.names.append("LWristYaw")
        self.times.append([0.04, 0.6, 1.56])
        self.keys.append(
            [[-0.100401, [3, -0.0133333, 0], [3, 0.186667, 0]], [-1.53059, [3, -0.186667, 0], [3, 0.32, 0]],
             [-1.53059, [3, -0.32, 0], [3, 0, 0]]])

        self.names.append("RElbowRoll")
        self.times.append([0.04, 0.6, 1.56])
        self.keys.append([[0.292386, [3, -0.0133333, 0], [3, 0.186667, 0]], [0.418879, [3, -0.186667, 0], [3, 0.32, 0]],
                          [0.418879, [3, -0.32, 0], [3, 0, 0]]])

        self.names.append("RElbowYaw")
        self.times.append([0.04, 0.6, 1.56])
        self.keys.append([[1.1802, [3, -0.0133333, 0], [3, 0.186667, 0]], [1.19381, [3, -0.186667, 0], [3, 0.32, 0]],
                          [1.19381, [3, -0.32, 0], [3, 0, 0]]])

        self.names.append("RHand")
        self.times.append([0.04, 0.6, 1.56])
        self.keys.append([[0.816771, [3, -0.0133333, 0], [3, 0.186667, 0]], [0.3, [3, -0.186667, 0], [3, 0.32, 0]],
                          [0.3, [3, -0.32, 0], [3, 0, 0]]])

        self.names.append("RShoulderPitch")
        self.times.append([0.04, 0.6, 1.56])
        self.keys.append([[1.30951, [3, -0.0133333, 0], [3, 0.186667, 0]],
                          [1.47131, [3, -0.186667, -7.76755e-08], [3, 0.32, 1.33158e-07]],
                          [1.47131, [3, -0.32, 0], [3, 0, 0]]])

        self.names.append("RShoulderRoll")
        self.times.append([0.04, 0.6, 1.56])
        self.keys.append(
            [[-0.128319, [3, -0.0133333, 0], [3, 0.186667, 0]], [-0.185005, [3, -0.186667, 0], [3, 0.32, 0]],
             [-0.185005, [3, -0.32, 0], [3, 0, 0]]])

        self.names.append("RWristYaw")
        self.times.append([0.04, 0.6, 1.56])
        self.keys.append(
            [[0.100401, [3, -0.0133333, 0], [3, 0.186667, 0]], [0.0994838, [3, -0.186667, 0], [3, 0.32, 0]],
             [0.0994838, [3, -0.32, 0], [3, 0, 0]]])

    def run(self):
        self.movementProxy.angleInterpolationBezier(self.names, self.times, self.keys)
