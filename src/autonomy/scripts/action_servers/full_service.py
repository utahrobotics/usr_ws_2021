#! /usr/bin/env python

import rospy
import os
import importlib


if __name__ == "__main__":
    rospy.init_node("actions")
    services = os.listdir("src/autonomy/scripts/action_servers")
    services.remove("full_service.py")
    services.remove("abstract_service.py")

    servers = []

    for service_path in services:
        if service_path.endswith("pyc"): continue
        service = importlib.import_module(service_path.split(".")[0])

        if not hasattr(service, "servers"):
            raise ImportError(
                "The module: " + service_path + " did not contain the array: servers" +
                "\n\tRemember to import abstract_service like this: from abstract_service import *"
            )

        for server_cls in service.servers:
            servers.append(server_cls())

    for server in servers:
        server.start()
        rospy.loginfo(server.name + " started")
