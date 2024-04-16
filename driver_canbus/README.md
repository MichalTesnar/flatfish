# Drivers CAN Bus

**DISCLAIMER**:
This is a fork/rework from [drivers-canbus](https://github.com/rock-drivers/drivers-canbus) to be able to use it as basis for ROS2 Driver development

**Maintainer:**
  - Miguel Bande Firvida <miguel.bande_firvida@dfki.de>
<!--- protected region package header ends -->


## Introduction

A generic implementation of a packet extraction algorithm on CAN bus.

## How to use

### CAN bus Socket

<!-- Note from the authors -->
When using the socket API, make sure the can network interface is up.
Using wireshark on the can network interface is possible, but it lacks a dissector. The raw data decodes like this:
- can id: 4 byte in host byte order. See `/usr/include/linux/can.h` for decoding the high 3 bits and `/usr/include/linux/can/error.h` for error packets.
- length: 1 byte
- garbage?: 3 byte
- can data: 8 byte
