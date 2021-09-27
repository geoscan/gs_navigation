# Описание пакета gs_navigation

## Описание:
Данный пакет предоставляет инструменты для работы с ситемами позиционирования

## Состав пакета:
Классы:
* LocalNavigation (приватный)
* OpticalFlow (приватный)
* NavigationManager

## Описание классов:
### 1. LocalNavigation
Класс взаимодействия с локальной системой навигации

#### Инициализация:
LocalNavigation(alive, navSystem, namespace) - alive и navSystem сервисы

#### Поля:
* name - str
* __namespace - str
* __alive - rospy.ServiceProxy: gs_interfaces.srv.Live
* __nav_service - rospy.ServiceProxy: gs_interfaces.srv.NavigationSystem
* __local_position - geometry_msgs.msg.Point
* __local_status - int
* __local_yaw - float
* __local_position_subscriber - rospy.Subscriber: geometry_msgs.msg.Point
* __local_status_subscriber - rospy.Subscriber: std_msgs.msg.Int32
* __local_yaw_subscriber - rospy.Subscriber: std_msgs.msg.Float32

#### Методы:
* position - получить локальные координаты LPS
* status - возвращает статус LPS
* yaw - возвращает угол поворота в системе LPS

#### Используемые топики:
* geoscan/navigation/local/position (geometry_msgs/Point)
* geoscan/navigation/local/status (std_msgs/Int32)
* geoscan/navigation/local/yaw (std_msgs/Float32)

### 2. OpticalFlow
Класс взаимодействия с модулем оптического потока (OPT)

#### Инициализация:
OpticalFlow(alive, navSystem, namespace) - alive и navSystem сервисы

#### Поля:
* name - str
* __namespace - str
* __alive - rospy.ServiceProxy: gs_interfaces.srv.Live
* __nav_service - rospy.ServiceProxy: gs_interfaces.srv.NavigationSystem
* __opt_velocity - gs_interfaces.msg.OptVelocity
* __opt_velocity_subscriber - rospy.Subscriber: gs_interfaces.msg.OptVelocity

#### Методы:
* velocity -получить информацию с модуля оптического потока (OPT)

#### Используемые топики:
* geoscan/navigation/opt/velocity (gs_interfaces/OptVelocity)

### 3. NavigationManager
Класс взаимодействия с информацией, получаемой от систем позиционирования

#### Инициализация:
NavigationManager(namespace = "") - namespace пространство имен в котором Пионер

#### Поля:
* __alive - rospy.ServiceProxy: gs_interfaces.srv.Live
* __nav_service - rospy.ServiceProxy: gs_interfaces.srv.NavigationSystem
* __set_nav_service - rospy.ServiceProxy: gs_interfaces.srv.SetNavigationSystem
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