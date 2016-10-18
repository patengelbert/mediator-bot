from naoqi import ALProxy
import time

port = 35721

motion = ALProxy("ALMotion", "localhost", port)
motion.post.stiffnessInterpolation("Body", 1, 0.1)
time.sleep(0.1)

postureProxy = ALProxy("ALRobotPosture", "localhost", port)
postureProxy.setMaxTryNumber(0)

postureProxy.goToPosture("Stand", 1)

time.sleep(0.1)

motion.moveInit()
motion.post.moveTo(1, 0, 0)

time.sleep(0.1)

postureProxy.goToPosture("Crouch", 1)

