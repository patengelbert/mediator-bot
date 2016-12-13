#! /usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import *

def client(name):
  rospy.wait_for_service('registerPerson')
  try:
    registerPerson=rospy.ServiceProxy('registerPerson', StartEnrollment)
    return true
  except rospy.ServiceException, e:
    print "exception"

if __name__ =="__main__":
  if len(sys.argv)==2:
    name= sys.argv[1]
  else: 
    print "lol"
    sys.exit(1)
  print name
  client(name)
