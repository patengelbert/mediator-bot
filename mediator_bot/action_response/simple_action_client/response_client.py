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

    goal = responseGoal(action="stop", name="Matthew", direction=0)
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

#    goal = responseGoal(action="stop", name="Mark", direction=45)
 #   client.send_goal(goal)
  #  client.wait_for_result(rospy.Duration.from_sec(5.0))

   # goal = responseGoal(action="start", name="Luke", direction=90)
#    client.send_goal(goal)
 #   client.wait_for_result(rospy.Duration.from_sec(5.0))

  #  goal = responseGoal(action="start", name="John", direction=-45)
   # client.send_goal(goal)
   # client.wait_for_result(rospy.Duration.from_sec(5.0))

#    goal = responseGoal(action="start", name="Matthew", direction=-90)
 #   client.send_goal(goal)
  #  client.wait_for_result(rospy.Duration.from_sec(5.0))

   # goal = responseGoal(action="start", name="Mark", direction=0)
    #client.send_goal(goal)
#    client.wait_for_result(rospy.Duration.from_sec(5.0))

 #   goal = responseGoal(action="loud", name="Luke", direction=45)
   # client.send_goal(goal)
  #  client.wait_for_result(rospy.Duration.from_sec(5.0))

    #goal = responseGoal(action="loud", name="John", direction=90)
#    client.send_goal(goal)
 #   client.wait_for_result(rospy.Duration.from_sec(5.0))
#
 #   goal = responseGoal(action="loud", name="Matthew", direction=-45)
  #  client.send_goal(goal)
   # client.wait_for_result(rospy.Duration.from_sec(5.0))

    #goal = responseGoal(action="loud", name="Mark", direction=-90)
#    client.send_goal(goal)
 #   client.wait_for_result(rospy.Duration.from_sec(5.0))
#
 #   goal = responseGoal(action="multiple", name="Luke", direction=0)
  #  client.send_goal(goal)
   # client.wait_for_result(rospy.Duration.from_sec(5.0))

    #goal = responseGoal(action="multiple", name="John", direction=45)
#    client.send_goal(goal)
 #   client.wait_for_result(rospy.Duration.from_sec(5.0))

    #goal = responseGoal(action="natural", direction=-30)
    #client.send_goal(goal)
    #client.wait_for_result(rospy.Duration.from_sec(5.0))

    #goal = responseGoal(action="start", name="Mark", direction=45)
    #client.send_goal(goal)
    #client.wait_for_result(rospy.Duration.from_sec(5.0))

    #goal = responseGoal(action="multiple")
    #client.send_goal(goal)
    #client.wait_for_result(rospy.Duration.from_sec(5.0))

    #goal = responseGoal(action="loud")
    #client.send_goal(goal)
    #client.wait_for_result(rospy.Duration.from_sec(5.0))

