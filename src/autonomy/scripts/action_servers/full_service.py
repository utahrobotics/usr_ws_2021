#! /usr/bin/env python

import rospy
import os
import importlib
from abstract_service import AbstractActionServer
from abc import ABCMeta


if __name__ == "__main__":
    rospy.init_node("actions")
    services = os.listdir("src/autonomy/scripts/action_servers")
    services.remove("full_service.py")
    services.remove("abstract_service.py")
    servers = []

    for service_path in services:
        if service_path.endswith("pyc"): continue
        service = importlib.import_module(service_path.split(".")[0])

        for item_name in dir(service):
            if item_name in ("ABCMeta", "AbstractActionServer"): continue
            cls = service.__getattribute__(item_name)
            if type(cls) != ABCMeta or cls.__base__ != AbstractActionServer: continue
            # THe constructor of the server should not have parameters
            servers.append(cls())

    for server in servers:
        server.start()
        rospy.loginfo(server.name + " started")
