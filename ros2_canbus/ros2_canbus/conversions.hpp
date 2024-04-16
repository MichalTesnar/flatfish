#ifndef CANBUS_ROS_CONVERSIONS_H
#define CANBUS_ROS_CONVERSIONS_H

#include <ros2_canbus/msg/canbus_message.hpp>
#include <drivers_canbus/Message.hpp>

#include "ros_utils.hpp"

namespace canbus {
    static inline ros2_canbus::msg::CanbusMessage canbusMessage2rosCanbusMsg(canbus::Message const& canbusMsg)
    {
        ros2_canbus::msg::CanbusMessage rosCanbusMsg;
        rosCanbusMsg.stamp = ros_utils::chronoTime2rosTime(canbusMsg.time);
        rosCanbusMsg.can_time = ros_utils::chronoTime2rosTime(canbusMsg.can_time);
        rosCanbusMsg.can_id = canbusMsg.can_id;
        memcpy(&rosCanbusMsg.data, &canbusMsg.data, 8);
        rosCanbusMsg.size = canbusMsg.size;
        return rosCanbusMsg;
    }

    static inline canbus::Message rosCanbusMsg2canbusMessage(ros2_canbus::msg::CanbusMessage const& rosCanbusMsg)
    {
        canbus::Message canbusMsg;
        canbusMsg.time = ros_utils::rosTime2chronoTime(rosCanbusMsg.stamp); // Note this is when sent it (?)
        canbusMsg.can_time = ros_utils::rosTime2chronoTime(rosCanbusMsg.can_time);  // Note this is when sent it (?)
        canbusMsg.can_id = rosCanbusMsg.can_id;
        memcpy(&canbusMsg.data, &rosCanbusMsg.data, 8);
	    canbusMsg.size = rosCanbusMsg.size;
        return canbusMsg;
    }
}


#endif
