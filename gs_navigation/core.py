#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from rospy import ServiceProxy,Subscriber
from gs_interfaces.srv import NavigationSystem,Live
from gs_interfaces.msg import PointGPS,OptVelocity
from geometry_msgs.msg import Point

class NavigationManager():
    def __local_position_callback(self,data):
        self.__local_position=data

    def __global_position_callback(self,data):
        self.__global_position=data

    def __opt_velocity_callback(self,data):
        self.__opt_velocity=data

    def __init__(self):
        self.__local_position=Point()
        self.__global_position=PointGPS()
        self.__opt_velocity=OptVelocity()
        rospy.wait_for_service("geoscan/alive")
        rospy.wait_for_service("geoscan/navigation/system")
        self.__alive=ServiceProxy("geoscan/alive",Live)
        self.__nav_service=ServiceProxy("geoscan/navigation/system",NavigationSystem)
        self.__local_position_sub=Subscriber("geoscan/local_position",Point,self.__local_position_callback)
        self.__global_position_sub=Subscriber("geoscan/global_position",PointGPS,self.__global_position_callback)
        self.__opt_velocity_sub=Subscriber("geoscan/opt_velocity",OptVelocity,self.__opt_velocity_callback)

    def navigationSystem(self):
        if(self.__alive().status):
            return self.__nav_service().navigation
        else:
            rospy.loginfo("Wait, connecting to flight controller")
            return self.error_number

    def globalPosition(self):
        return self.__global_position.latitude, self.__global_position.longitude, self.__global_position.altitude

    def localPosition(self):
        return self.__local_position.x, self.__local_position.y, self.__local_position.z

    def optVelocity(self):
        return self.__opt_velocity.x,self.__opt_velocity.y,self.__opt_velocity.height
