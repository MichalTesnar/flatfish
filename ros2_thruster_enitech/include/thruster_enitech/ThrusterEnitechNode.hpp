#ifndef ROS2_THRUSTER_ENITECH_NODE_HPP
#define ROS2_THRUSTER_ENITECH_NODE_HPP


#include <boost/shared_ptr.hpp>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "auv_control_msgs/msg/joint_command_stamped.hpp"
#include "thruster_enitech/msg/thruster_status.hpp"
#include "Protocol.hpp"
#include "SDO.hpp"
#include "NMT.hpp"

#include <ros2_canbus/conversions.hpp>
#include <ros2_canbus/msg/canbus_message.hpp>


namespace thruster_enitech
{

class ThrusterEnitechNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    ThrusterEnitechNode(const std::string& name = "thruster_enitech_node");
    ~ThrusterEnitechNode();

protected:
    //boost::shared_ptr<thruster_enitech::Protocol> m_protocol;
    thruster_enitech::Protocol m_protocol;

    std::string thruster_name_;
    int thruster_id_;

    // Time stamp of the last thruster command recieved
    rclcpp::Time last_thrust_cmd_ts_;
    // Time stamp of the last thruster status recieved
    rclcpp::Time last_status_ts_;

    // hceck if new command is received
    bool new_thruster_cmd_ = false;
    bool new_can_in_ = false;

    //BG149Temperature bg149_temperature; /**todo define this type*/

    // local variable storing received can message 
    canbus::Message can_in_;
    // local variable storing received thruster command
    auv_control_msgs::msg::JointCommandStamped thruster_cmd_;
    
    // class functions
    bool process_request(Request &request, const rclcpp::Duration timeout);
    int get_thruster_by_name(std::vector<std::string> const &names);

    // canbus subscriber
    std::shared_ptr<rclcpp::Subscription<ros2_canbus::msg::CanbusMessage>> can_subscriber_;
    // thruster command subscriber
    std::shared_ptr<rclcpp::Subscription<auv_control_msgs::msg::JointCommandStamped>> thruster_command_subscriber_;

    // canbus publisher
    rclcpp_lifecycle::LifecyclePublisher<ros2_canbus::msg::CanbusMessage>::SharedPtr can_publisher_;
    // thruster state publisher
    rclcpp_lifecycle::LifecyclePublisher<thruster_enitech::msg::ThrusterStatus>::SharedPtr thruster_state_publisher_;

    rclcpp::TimerBase::SharedPtr timer_update_;
    rclcpp::TimerBase::SharedPtr timer_status_timeout_;

    // callbacks
    void can_in_callback(const ros2_canbus::msg::CanbusMessage::SharedPtr can_in_msg);
    void thruster_command_callback(const auv_control_msgs::msg::JointCommandStamped::SharedPtr thruster_command_msg);
    void update_callback();
    void timer_status_timeout_callback();

    //
    rclcpp::CallbackGroup::SharedPtr cb_group_can;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure (const rclcpp_lifecycle::State& previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup (const rclcpp_lifecycle::State& previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate (const rclcpp_lifecycle::State& previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate (const rclcpp_lifecycle::State& previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown (const rclcpp_lifecycle::State& previous_state) override;
};

}  // end namespace thruster_enitech

#endif  // ROS2_THRUSTER_ENITECH_NODE_HPP
