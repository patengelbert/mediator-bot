#! /usr/bin/env python

class StopLeft:
    def __init__(self, angle, movementProxy):
        self.names = list()
        self.times = list()
        self.keys = list()

        self.movementProxy = movementProxy

        self.names.append("LElbowRoll")
        self.times.append([0.04, 0.6, 1.52])
        self.keys.append(
            [[-0.399604, [3, -0.0133333, 0], [3, 0.186667, 0]], [-0.724422, [3, -0.186667, 0], [3, 0.306667, 0]],
             [-0.724422, [3, -0.306667, 0], [3, 0, 0]]])

        self.names.append("LElbowYaw")
        self.times.append([0.04])
        self.keys.append([[-1.17765, [3, -0.0133333, 0], [3, 0, 0]]])

        self.names.append("LHand")
        self.times.append([0.04, 0.6, 1.52])
        self.keys.append([[0.6, [3, -0.0133333, 0], [3, 0.186667, 0]], [1, [3, -0.186667, 0], [3, 0.306667, 0]],
                          [1, [3, -0.306667, 0], [3, 0, 0]]])

        self.names.append("LShoulderPitch")
        self.times.append([0.04, 0.6, 1.52])
        self.keys.append([[1.3245, [3, -0.0133333, 0], [3, 0.186667, 0]],
                          [0.283669, [3, -0.186667, 4.86316e-07], [3, 0.306667, -7.98948e-07]],
                          [0.283668, [3, -0.306667, 0], [3, 0, 0]]])

        self.names.append("LShoulderRoll")
        self.times.append([0.04, 0.6, 1.52])

        self.keys.append([[0.118046, [3, -0.0133333, 0], [3, 0.186667, 0]], [angle, [3, -0.186667, 0], [3, 0.306667, 0]],
                          [angle, [3, -0.306667, 0], [3, 0, 0]]])

        self.names.append("LWristYaw")
        self.times.append([0.04, 0.6, 1.52])
        self.keys.append(
            [[-0.103232, [3, -0.0133333, 0], [3, 0.186667, 0]], [1.10515, [3, -0.186667, 0], [3, 0.306667, 0]],
             [1.10515, [3, -0.306667, 0], [3, 0, 0]]])

        self.names.append("RElbowRoll")
        self.times.append([0.04, 0.6, 1.52])
        self.keys.append(
            [[0.399604, [3, -0.0133333, 0], [3, 0.186667, 0]], [0.418879, [3, -0.186667, 0], [3, 0.306667, 0]],
             [0.418879, [3, -0.306667, 0], [3, 0, 0]]])

        self.names.append("RElbowYaw")
        self.times.append([0.04, 0.6, 1.52])
        self.keys.append(
            [[1.17765, [3, -0.0133333, 0], [3, 0.186667, 0]], [1.19381, [3, -0.186667, 0], [3, 0.306667, 0]],
             [1.19381, [3, -0.306667, 0], [3, 0, 0]]])

        self.names.append("RHand")
        self.times.append([0.04, 0.6, 1.52])
        self.keys.append([[0.6, [3, -0.0133333, 0], [3, 0.186667, 0]], [0.3, [3, -0.186667, 0], [3, 0.306667, 0]],
                          [0.3, [3, -0.306667, 0], [3, 0, 0]]])

        self.names.append("RShoulderPitch")
        self.times.append([0.04, 0.6, 1.52])
        self.keys.append([[1.3245, [3, -0.0133333, 0], [3, 0.186667, 0]],
                          [1.47131, [3, -0.186667, -8.10527e-08], [3, 0.306667, 1.33158e-07]],
                          [1.47131, [3, -0.306667, 0], [3, 0, 0]]])

        self.names.append("RShoulderRoll")
        self.times.append([0.04, 0.6, 1.52])
        self.keys.append([[-0.118046, [3, -0.0133333, 0], [3, 0.186667, 0]],
                          [-0.179769, [3, -0.186667, 1.01316e-08], [3, 0.306667, -1.66448e-08]],
                          [-0.179769, [3, -0.306667, 0], [3, 0, 0]]])

        self.names.append("RWristYaw")
        self.times.append([0.04, 0.6, 1.52])
        self.keys.append(
            [[0.103232, [3, -0.0133333, 0], [3, 0.186667, 0]], [0.0994838, [3, -0.186667, 0], [3, 0.306667, 0]],
             [0.0994838, [3, -0.306667, 0], [3, 0, 0]]])

    def run(self):
        self.movementProxy.angleInterpolationBezier(self.names, self.times, self.keys)
