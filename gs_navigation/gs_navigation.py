#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from rospy import ServiceProxy,Subscriber
from gs_interfaces.srv import NavigationSystem,SetNavigationSystem,Live
from gs_interfaces.msg import PointGPS,SatellitesGPS,OptVelocity
from geometry_msgs.msg import Point
from std_msgs.msg import Float32,Int8

class GlobalNavigation():
    def __global_position_callback(self, data):
        self.__global_position = data

    def __sattelites_callback(self, data):
        self.__satellites = data

    def __global_status_callback(self, data):
        self.__global_status = data

    def __init__(self, alive, navSystem):
        self.name = "GPS"
        self.__global_position = PointGPS()
        self.__satellites = SatellitesGPS()
        self.__global_status = Int8()
        self.__alive =  alive
        self.__nav_service = navSystem

        self.__global_position_subscriber = Subscriber("geoscan/navigation/global/position", PointGPS, self.__global_position_callback)
        self.__satellites_subscriber = Subscriber("geoscan/navigation/satellites", SatellitesGPS, self.__sattelites_callback)
        self.__global_status_subscriber = Subscriber("geoscan/navigation/global/status", Int8, self.__global_status_callback)

    def position(self):
        if self.__alive().status:
            if self.__nav_service().navigation == self.name:
                rospy.wait_for_message("geoscan/navigation/global/position", PointGPS)
            return self.__global_position.latitude, self.__global_position.longitude, self.__global_position.altitude
        else:
            rospy.logwarn("Wait, connecting to flight controller")

    def satellites(self):
        if self.__alive().status:
            if self.__nav_service().navigation == self.name:
                rospy.wait_for_message("geoscan/navigation/satellites", SatellitesGPS)
            return self.__satellites.gps, self.__satellites.glonass
        else:
            rospy.logwarn("Wait, connecting to flight controller")

    def status(self):
        if self.__alive().status:
            if self.__nav_service().navigation == self.name:
                rospy.wait_for_message("geoscan/navigation/global/status", Int8)
            return self.__global_status.data
        else:
            rospy.logwarn("Wait, connecting to flight controller")

class LocalNavigation():
    def __local_position_callback(self, data):
        self.__local_position = data

    def __local_velocity_callback(self, data):
        self.__local_velocity = data

    def __local_yaw_callback(self, data):
        self.__local_yaw = data.data

    def __init__(self, alive, navSystem):
        self.name = "LPS"
        self.__local_position = Point()
        self.__local_velocity = Point()
        self.__local_yaw = 0.0

        self.__alive =  alive
        self.__nav_service = navSystem
        self.__local_position_subscriber = Subscriber("geoscan/navigation/local/position", Point, self.__local_position_callback)
        self.__local_velocity_subscriber = Subscriber("geoscan/navigation/local/velocity", Point, self.__local_velocity_callback)
        self.__local_yaw_subscriber = Subscriber("geoscan/navigation/local/yaw", Float32, self.__local_yaw_callback)

    def position(self):
        if self.__alive().status:
            if self.__nav_service().navigation == self.name:
                rospy.wait_for_message("geoscan/navigation/local/position", Point)
            return self.__local_position.x, self.__local_position.y, self.__local_position.z
        else:
            rospy.logwarn("Wait, connecting to flight controller")

    def velocity(self):
        if self.__alive().status:
            if self.__nav_service().navigation == self.name:
                rospy.wait_for_message("geoscan/navigation/local/velocity", Point)
            return self.__local_velocity.x, self.__local_velocity.y, self.__local_velocity.z
        else:
            rospy.logwarn("Wait, connecting to flight controller")
        
    def yaw(self):
        if self.__alive().status:
            if self.__nav_service().navigation == self.name:
                rospy.wait_for_message("geoscan/navigation/local/yaw", Float32)
            return self.__local_yaw
        else:
            rospy.logwarn("Wait, connecting to flight controller")

class OpticalFlow():
    def __opt_velocity_callback(self, data):
        self.__opt_velocity = data

    def __init__(self, alive, navSystem):
        self.name = "OPT"

        self.__opt_velocity = OptVelocity()
        self.__alive =  alive
        self.__nav_service = navSystem

        self.__opt_velocity_subscriber = Subscriber("geoscan/navigation/opt/velocity", OptVelocity, self.__opt_velocity_callback)

    def velocity(self):
        if self.__alive().status:
            if self.__nav_service().navigation == self.name:
                rospy.wait_for_message("geoscan/navigation/opt/velocity", OptVelocity)
            return self.__opt_velocity.x, self.__opt_velocity.y, self.__opt_velocity.range
        else:
            rospy.logwarn("Wait, connecting to flight controller")

class NavigationManager():
    def __init__(self):
        rospy.wait_for_service("geoscan/alive")
        rospy.wait_for_service("geoscan/navigation/get_system")
        rospy.wait_for_service("geoscan/navigation/set_system")
        self.__alive = ServiceProxy("geoscan/alive", Live)
        self.__nav_service = ServiceProxy("geoscan/navigation/get_system", NavigationSystem)
        self.__set_nav_service = ServiceProxy("geoscan/navigation/set_system", SetNavigationSystem)

        self.gps = GlobalNavigation(self.__alive, self.__nav_service)
        self.lps = LocalNavigation(self.__alive, self.__nav_service)
        self.opt = OpticalFlow(self.__alive, self.__nav_service)

    def system(self):
        if self.__alive().status:
            return self.__nav_service().navigation
        else:
            rospy.logwarn("Wait, connecting to flight controller")

    def setSystem(self, system):
        if (system == "OPT") or (system == "GPS") or (system == "LPS"):
            return self.__set_nav_service(system).status
        else:
            return False