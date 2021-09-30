#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from rospy import ServiceProxy,Subscriber
from gs_interfaces.srv import NavigationSystem, SetNavigationSystem, Live
from gs_interfaces.msg import OptVelocity
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Int32

class LocalNavigation():
    def __local_position_callback(self, data):
        self.__local_position = data

    def __local_yaw_callback(self, data):
        self.__local_yaw = data.data

    def __local_status_callback(self, data):
        self.__local_status = data.data

    def __init__(self, alive, navSystem, namespace):
        self.name = "LPS"
        self.__namespace = namespace

        self.__local_position = Point()
        self.__local_status = 0
        self.__local_yaw = 0.0

        self.__alive =  alive
        self.__nav_service = navSystem
        self.__local_position_subscriber = Subscriber(f"{self.__namespace}geoscan/navigation/local/position", Point, self.__local_position_callback)
        self.__local_status_subscriber = Subscriber(f"{self.__namespace}geoscan/navigation/local/status", Int32, self.__local_status_callback)
        self.__local_yaw_subscriber = Subscriber(f"{self.__namespace}geoscan/navigation/local/yaw", Float32, self.__local_yaw_callback)

    def position(self):
        if self.__alive().status:
            return self.__local_position.x, self.__local_position.y, self.__local_position.z
        else:
            rospy.logwarn("Wait, connecting to flight controller")

    def status(self):
        if self.__alive().status:
            return self.__local_status
        else:
            rospy.logwarn("Wait, connecting to flight controller")
        
    def yaw(self):
        if self.__alive().status:
            return self.__local_yaw
        else:
            rospy.logwarn("Wait, connecting to flight controller")

class OpticalFlow():
    def __opt_velocity_callback(self,data):
        self.__opt_velocity = data

    def __init__(self, alive, navSystem, namespace):
        self.name = "OPT"
        self.__namespace = namespace

        self.__opt_velocity = OptVelocity()
        self.__alive =  alive
        self.__nav_service = navSystem

        self.__opt_velocity_subscriber = Subscriber(f"{self.__namespace}geoscan/navigation/opt/velocity", OptVelocity, self.__opt_velocity_callback)

    def velocity(self):
        if self.__alive().status:
            return self.__opt_velocity.x, self.__opt_velocity.y, self.__opt_velocity.range
        else:
            rospy.logwarn("Wait, connecting to flight controller")

class NavigationManager():
    def __init__(self, namespace = ""):
        if namespace != "":
            namespace += "/"
        rospy.wait_for_service(f"{namespace}geoscan/alive")
        rospy.wait_for_service(f"{namespace}geoscan/navigation/get_system")
        rospy.wait_for_service(f"{namespace}geoscan/navigation/set_system")
        self.__alive = ServiceProxy(f"{namespace}geoscan/alive", Live)
        self.__nav_service = ServiceProxy(f"{namespace}geoscan/navigation/get_system", NavigationSystem)
        self.__set_nav_service = ServiceProxy(f"{namespace}geoscan/navigation/set_system", SetNavigationSystem)

        self.lps = LocalNavigation(self.__alive, self.__nav_service, namespace)
        self.opt = OpticalFlow(self.__alive, self.__nav_service, namespace)

    def system(self):
        if self.__alive().status:
            return self.__nav_service().navigation
        else:
            rospy.logwarn("Wait, connecting to flight controller")

    def setSystem(self, system):
        if ((system == "OPT") or (system == "LPS")):
            return self.__set_nav_service(system).status
        else:
            return False