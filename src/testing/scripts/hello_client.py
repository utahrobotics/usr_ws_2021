#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from testing.srv import *

def hello_client(name):
    rospy.wait_for_service('hello')
    try:
        hello_srv = rospy.ServiceProxy('hello', hello)
        resp1 = hello_srv(name)
        return resp1.response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [name]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        name = sys.argv[1]
    else:
        print(usage())
        sys.exit(1)
    print("%s"%(hello_client(name)))
