#!/usr/bin/env python

from beginner_tutorials.srv import *
import rospy

def handle_register(req):
  print "returning true"
  return StartEnrollmentResponse(True);

def register_server():
  rospy.init_node('register_server')
  print "ready"
  s=rospy.Service('/StartEnrollment',StartEnrollment, handle_register)
  rospy.spin()

if __name__ == "__main__":
  register_server() 
