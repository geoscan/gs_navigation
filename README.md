# Описание пакета gs_navigation

## Описание:
Данный пакет предоставляет инструменты для работы с ситемами позиционирования

## Состав пакета:
Классы:
* GlobalNavigation (приватный)
* LocalNavigation (приватный)
* OpticalFlow (приватный)
* NavigationManager

## Описание классов:

### 1. GlobalNavigation
Класс взаимодействия с глобальной системой навигации

#### Инициализация:
GlobalNavigation(alive, navSystem) - alive и navSystem сервисы

#### Поля:
* name - str
* __alive - rospy.ServiceProxy: gs_interfaces.srv.Live
* __nav_service - rospy.ServiceProxy: gs_interfaces.srv.NavigationSystem
* __global_position - gs_interfaces.msg.PointGPS
* __satellites - gs_interfaces.msg.SatellitesGPS
* __global_status - std_msgs.msg.Int8
* __global_position_subscriber - rospy.Subscriber: gs_interfaces.msg.PointGPS
* __satellites_subscriber - rospy.Subscriber: gs_interfaces.msg.SatellitesGPS
* __global_status_subscriber - rospy.Subscriber: std_msgs.msg.Int8

#### Методы:
* position - получить глобальные координаты GPS
* satellites - получить количество спутников
* status - получить статус системы позиционирования

#### Используемые топики:
* geoscan/navigation/global/position (gs_interfaces/PointGPS)
* geoscan/navigation/satellites (gs_interfaces/SatellitesGPS)
* geoscan/navigation/global/status (std_msgs/Int8)

### 2. LocalNavigation
Класс взаимодействия с локальной системой навигации

#### Инициализация:
LocalNavigation(alive, navSystem) - alive и navSystem сервисы

#### Поля:
* name - str
* __alive - rospy.ServiceProxy: gs_interfaces.srv.Live
* __nav_service - rospy.ServiceProxy: gs_interfaces.srv.NavigationSystem
* __local_position - geometry_msgs.msg.Point
* __local_velocity - geometry_msgs.msg.Point
* __local_yaw - int
* __local_position_subscriber - rospy.Subscriber: geometry_msgs.msg.Point
* __local_velocity_subscriber - rospy.Subscriber: geometry_msgs.msg.Point
* __local_yaw_subscriber - rospy.Subscriber: std_msgs.msg.Float32

#### Методы:
* position - получить локальные координаты LPS
* velocity - возвращает скорость коптера возвращаемую LPS (vx,vy,vz)
* yaw - возвращает угол поворота в системе LPS

#### Используемые топики:
* geoscan/navigation/local/position (geometry_msgs/Point)
* geoscan/navigation/local/velocity (geometry_msgs/Point)
* geoscan/navigation/local/yaw (std_msgs/Float32)

### 3. OpticalFlow
Класс взаимодействия с модулем оптического потока (OPT)

#### Инициализация:
OpticalFlow(alive, navSystem) - alive и navSystem сервисы

#### Поля:
* name - str
* __alive - rospy.ServiceProxy: gs_interfaces.srv.Live
* __nav_service - rospy.ServiceProxy: gs_interfaces.srv.NavigationSystem
* __opt_velocity - gs_interfaces.msg.OptVelocity
* __opt_velocity_subscriber - rospy.Subscriber: gs_interfaces.msg.OptVelocity

#### Методы:
* velocity -получить информацию с модуля оптического потока (OPT)

#### Используемые топики:
* geoscan/navigation/opt/velocity (gs_interfaces/OptVelocity)

### 4. NavigationManager
Класс взаимодействия с информацией, получаемой от систем позиционирования

#### Инициализация:
Без параметров

#### Поля:
* __alive - rospy.ServiceProxy: gs_interfaces.srv.Live
* __nav_service - rospy.ServiceProxy: gs_interfaces.srv.NavigationSystem
* __set_nav_service - rospy.ServiceProxy: gs_interfaces.srv.SetNavigationSystem
* gps - GlobalNavigation
* lps - LocalNavigation
* opt - OpticalFlow

#### Методы:
* system - получить текущую систему позиционирования
* setSystem(system) - установить систему позиционирования, system - название системы позиционирования

#### Используемые сервисы:
* geoscan/alive (gs_interfaces/Live)
* geoscan/navigation/get_system (gs_interfaces/NavigationSystem)
* geoscan/navigation/set_system (gs_interfaces/SetNavigationSystem)

## Необходимые пакеты:
ROS:
* gs_interfaces
* gs_core
* geometry_msgs
* std_msgs

## Примечание:
Все классы в данном пакете могут быть использованы только при запущеной ноде ros_plaz_node.py из пакета gs_core