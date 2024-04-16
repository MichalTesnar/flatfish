# ros2_driver_base

DISCLAIMER: This is a fork / rework from [ros_driver_base](https://github.com/Brazilian-Institute-of-Robotics/ros_driver_base) to be able to use it as basis for ROS2 Driver development

## Purpose

A generic implementation of a packet extraction algorithm on an I/O device.

## How to use

This class provides the basic service for reading an I/O device until a full
packet has been read, and returning that packet. It does so while maintaining
a proper read and write timeout.

To use this class:
- subclass it
- give to the Driver constructor the maximum packet size that it can expect
- implement extractPacket (see below)

Then, you can freely use writePacket() and readPacket() to write/read data from
the device.

The issue that this class is trying to solve in a generic way is that, when
reading on I/O, one will seldom read a full packet at once. What this class
does is to accumulate data in readPacket, until the subclass-provided
extractPacket implementation finds a packet in the buffer. When a packet is
found, it is copied into the buffer given to readPacket and the packet size
is returned.

See extractPacket() documentation for more information on how to implement this method.