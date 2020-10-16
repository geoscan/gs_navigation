# Описание пакета gs_navigation

## Описание:
Данный пакет предоставляет инструменты для работы с ситемами позиционирования

## Состав пакета:
Классы:
* NavigationManager

## Описание классов:

### 1. NavigationManager
Класс взаимодействия с информацией, получаемой от систем позиционирования

#### Инициализация:
Без параметров

#### Поля:
* __alive - rospy.ServiceProxy: gs_interfaces.srv.Live
* __local_position - geometry_msgs.msg.Point
* __global_position - gs_interfaces.msg.PointGPS
* __opt_velocity - gs_interfaces.msg.OptVelocity
* __nav_service - rospy.ServiceProxy: gs_interfaces.srv.NavigationSystem
* __local_position_sub - rospy.Subscriber: geometry_msgs.msg.Point
* __global_position_sub - rospy.Subscriber: gs_interfaces.msg.PointGPS
* __opt_velocity_sub - rospy.Subscriber: gs_interfaces.msg.OptVelocity

#### Методы:
* navigationSystem() - получить текущую систему навигации
* globalPosition() - получить глобальные координаты GPS
* localPosition() - получить локальные координаты LPS
* optVelocity() -получить информацию с модуля оптического потока (OPT)

#### Используемые сервисы:
* geoscan/alive (gs_interfaces/Live
* geoscan/navigation/system (gs_interfaces/NavigationSystem)

#### Используемые топики:
* geoscan/local_position (geometry_msgs/Point)
* geoscan/global_position (gs_interfaces/PointGPS)
* geoscan/opt_velocity (gs_interfaces/OptVelocity)

## Необходимые пакеты:
ROS:
* gs_interfaces
* gs_core

## Примечание:
Все классы в данном пакете могут быть использованы только при запущеной ноде ros_serial_node.py из пакета gs_core