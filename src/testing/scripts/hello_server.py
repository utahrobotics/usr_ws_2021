#!/usr/bin/env python

from __future__ import print_function

from testing.srv import hello,helloResponse
import rospy

def handle_hello(req):
    print("helloing")
    rospy.logdebug("helloing")
    return helloResponse("hello "+req.name)

def hello_server():
    rospy.init_node('hello_server')
    rospy.logdebug("reeeee")
    s = rospy.Service('hello', hello, handle_hello)
    print("Ready to respond")
    rospy.spin()

if __name__ == "__main__":
    hello_server()
