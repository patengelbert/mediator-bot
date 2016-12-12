#! /usr/bin/env python

import roslib
roslib.load_manifest('action_response')
import rospy
import actionlib

from action_response.msg import responseAction, responseGoal

if __name__ == '__main__':
    rospy.init_node('response_client')
    client = actionlib.SimpleActionClient('response', responseAction)
    client.wait_for_server()

    goal = responseGoal(keywords=["stop"], name="Matthew", direction=0)
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["stop"], name="Mark", direction=45)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["start"], name="Luke", direction=90)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["start"], name="John", direction=-45)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["start"], name="Matthew", direction=-90)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["start"], name="Mark", direction=0)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["loud"], name="Luke", direction=45)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["loud"], name="John", direction=90)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["loud"], name="Matthew", direction=-45)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["loud"], name="Mark", direction=-90)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["multiple"], name="Luke", direction=0)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["multiple"], name="John", direction=45)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["natural"], direction=-30)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords="start", name="Mark", direction=45)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["multiple"])
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    goal = responseGoal(keywords=["loud"])
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
