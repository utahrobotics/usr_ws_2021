#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import errno
from testing.srv import audio

def audio_client(time):
    rospy.wait_for_service('Audio')
    try:
        audio_srv = rospy.ServiceProxy('Audio', audio)
        resp1 = audio_srv(time)
        return resp1.filePath
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [time]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        time = float(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("%s"%(audio_client(time)))
