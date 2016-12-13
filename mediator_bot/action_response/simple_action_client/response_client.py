#! /usr/bin/env python

import roslib
import time

roslib.load_manifest('action_response')
import rospy
import actionlib

from action_response.msg import responseAction, responseGoal

if __name__ == '__main__':
    rospy.init_node('response_client')
    client = actionlib.SimpleActionClient('response', responseAction)
    client.wait_for_server()

    goal = responseGoal(keywords=["intro"], name="Meng", direction=45)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["look"], name="Meng", direction=45)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["look"], name="Mark", direction=20)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["look"], name="Mark", direction=-40)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["stop", "polite"], name="Mark", direction=45)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)

    goal = responseGoal(keywords=["start"], name="Luke", direction=90)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)

    goal = responseGoal(keywords=["start"], name="John", direction=-45)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)

    goal = responseGoal(keywords=["start"], name="Matthew", direction=-90)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)

    goal = responseGoal(keywords=["start"], name="Mark", direction=0)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)

    goal = responseGoal(keywords=["loud"], name="Luke", direction=45)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)

    goal = responseGoal(keywords=["loud"], name="John", direction=90)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)

    goal = responseGoal(keywords=["loud"], name="Matthew", direction=-45)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)

    goal = responseGoal(keywords=["loud"], name="Mark", direction=-90)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)

    goal = responseGoal(keywords=["multiple"], name="Luke", direction=0)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)

    goal = responseGoal(keywords=["multiple"], name="John", direction=45)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)

    goal = responseGoal(keywords=["natural"], direction=-30)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)

    goal = responseGoal(keywords="start", name="Mark", direction=45)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)

    goal = responseGoal(keywords=["multiple"])
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)

    goal = responseGoal(keywords=["loud"])
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)

    goal = responseGoal(keywords=["nearly_done"], name="Meng", direction=0)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)

    goal = responseGoal(keywords=["outro"], name="Meng", direction=45)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    time.sleep(2)
