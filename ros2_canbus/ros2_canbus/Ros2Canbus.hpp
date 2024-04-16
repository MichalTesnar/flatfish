/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CANBUS_TASK_TASK_HPP
#define CANBUS_TASK_TASK_HPP

// std libs
#include <iostream>
#include <queue>
#include <vector>

// ROS2 libs
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
// #include <rclcpp/clock.hpp>


// ROS2 custom messages
#include <ros2_canbus/msg/canbus_message.hpp>
#include <ros2_canbus/msg/array_canbus_message.hpp>
#include <ros2_canbus/msg/canbus_statistics.hpp>

#include <ros2_canbus/srv/watch.hpp>
#include <ros2_canbus/srv/unwatch.hpp>
#include <ros2_canbus/srv/get_topic_name.hpp>
#include <ros2_canbus/srv/is_watched.hpp>


// own libs
#include <drivers_canbus/Driver.hpp>
#include <conversions.hpp>
#include <ros_utils.hpp>

// parameter header
// #include <canbus_parameters.hpp>

#define QOS_HISTORY_DEPTH 10

namespace ros2_canbus {
    class Ros2CanbusNode : public rclcpp_lifecycle::LifecycleNode
    {
    protected:

        typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface Lifecycle;

        struct DeviceMapping
        {
            std::string name;
            uint32_t id;
            // uint32_t mask;
            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ros2_canbus::msg::CanbusMessage>> publisher; //interface
            std::shared_ptr<rclcpp::Subscription<ros2_canbus::msg::CanbusMessage>> subscription;
            bool newMessage = false; // flag to publish only newest messages
            ros2_canbus::msg::CanbusMessage message;
        };
        typedef std::vector<DeviceMapping> DeviceMappings;

    private:

        std::string m_node_name;
        canbus::Driver *m_driver; /**<  */
        ros2_canbus::msg::CanbusStatistics m_stats; /**<  */
        DeviceMappings m_watched_devices; /**<  */
        ros2_canbus::msg::ArrayCanbusMessage m_unwatched_devices; /**<  */
        // MappingCache m_mapping_cache; /**<  */
        uint32_t m_global_mask = 0;

        std::shared_ptr<rclcpp::TimerBase> m_update_timer; /**<  Timer declaration for system update */
        std::shared_ptr<rclcpp::TimerBase> m_check_bus_ok_timer; /**<  Timer declaration to check if bus has errors */
        std::shared_ptr<rclcpp::TimerBase> m_publish_statistics_timer; /**<  Timer declaration for  */


        // Declaration of the publisher_ attribute
        //
        // We hold an instance of a lifecycle publisher. This lifecycle publisher
        // can be activated or deactivated regarding on which state the lifecycle node
        // is in.
        // By default, a lifecycle publisher is inactive by creation and has to be
        // activated to publish messages into the ROS world.
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ros2_canbus::msg::CanbusMessage>> m_pub_msgs_from_unwatched_device;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ros2_canbus::msg::ArrayCanbusMessage>> m_pub_msgs_from_unwatched_devices;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ros2_canbus::msg::CanbusStatistics>> m_pub_statistics;


        // Any message or messages received in this way, will be published
        // independent of whether they are watch or not (?)
        std::shared_ptr<rclcpp::Subscription<ros2_canbus::msg::CanbusMessage>> m_sub_tx_message;
        std::shared_ptr<rclcpp::Subscription<ros2_canbus::msg::ArrayCanbusMessage>> m_sub_tx_messages;
        // std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::ros2_canbus::msg::TransitionEvent>> m_sub_notification;

        rclcpp::Service<srv::Watch>::SharedPtr m_srv_watch;
        rclcpp::Service<srv::Unwatch>::SharedPtr m_srv_unwatch;
        rclcpp::Service<srv::GetTopicName>::SharedPtr m_srv_get_topic_name;
        rclcpp::Service<srv::IsWatched>::SharedPtr m_srv_is_watched;
        //
        // Note: we don't send the messages in the callback as we have to make sure that they are sent it (security)
        // Instead we stored them in a "queue" (this case we use directly the ROS msgs array) and then send them in the update loop
        std::queue<canbus::Message> m_queue_tx_messages; /**< A queue of messages to be sent that where received during the last period */
        // std::queue<ros2_canbus::msg::ArrayCanbusMessage> m_queue_rx_messages; /**< A queue of messages to be sent that where received during the last period */


        void m_setRosNode();
        void m_unsetRosNode();
        void m_activateRosNode();
        void m_deactivateRosNode();

        void m_declareRosParameters();

        void m_createSubscriptions();
        void m_activateSubscriptions();
        void m_deactivateSubscriptions();
        void m_deleteSubscriptions();

        void m_createPublishers();
        void m_activatePublishers();
        void m_deactivatePublishers();
        void m_deletePublishers();

        void m_createServices();
        void m_activateServices();
        void m_deactivateServices();
        void m_deleteServices();

        void m_activatePublishersOfWatchedDevices();
        void m_deactivatePublishersOfWatchedDevices();

        void m_createTimers();
        void m_activateTimers();
        void m_deactivateTimers();
        void m_deleteTimers();

        bool m_watch(std::string const& name, uint32_t id);

        void m_unwatchAll();

        DeviceMappings::iterator m_unwatch(DeviceMappings::iterator watched_device_it);
        bool m_unwatch(const uint32_t id);
        // bool m_unwatch(std::string const& name);

        int m_findWatchedDevice(const uint32_t id);
        bool m_isWatched(std::string const& name);
        bool m_isWatched(const uint32_t id);

        bool m_openCanPort(std::string const& device, const canbus::DRIVER_TYPE device_type);
        bool m_resetCanPort();
        void m_closeCanPort();

        void m_readCanbus();
        void m_writeCanbus();

        void m_publishCanbusMsgsFromWatchedDevices();
        void m_publishCanbusMsgsFromUnwatchedDevices();
        void m_publishCanbusStatistics();

    public:
        Ros2CanbusNode(std::string const& name = "canbus", bool intra_process_comms = false);
        ~Ros2CanbusNode();


        Lifecycle::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        Lifecycle::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        Lifecycle::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        Lifecycle::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        Lifecycle::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;


        void on_update();
        void on_checkBusOk();
        void on_publishStatistics();

        void cb_txMessage(ros2_canbus::msg::CanbusMessage::ConstSharedPtr msg);
        void cb_txMessages(ros2_canbus::msg::ArrayCanbusMessage::ConstSharedPtr msg);
        void cb_txMessageFromWatched(ros2_canbus::msg::CanbusMessage::ConstSharedPtr msg, const uint32_t can_id);

        void cb_watch
            (const std::shared_ptr<ros2_canbus::srv::Watch::Request>     request,
             std::shared_ptr<ros2_canbus::srv::Watch::Response>          response);
        void cb_unwatch
            (const std::shared_ptr<ros2_canbus::srv::Unwatch::Request>   request,
             std::shared_ptr<ros2_canbus::srv::Unwatch::Response>        response);
        void cb_get_topic_name
            (const std::shared_ptr<ros2_canbus::srv::GetTopicName::Request>  request,
             std::shared_ptr<ros2_canbus::srv::GetTopicName::Response>   response);
        void cb_is_watched
            (const std::shared_ptr<ros2_canbus::srv::IsWatched::Request> request,
             std::shared_ptr<ros2_canbus::srv::IsWatched::Response>      response);

    };
}

#endif
