#include <ThrusterEnitechNode.hpp>

using namespace thruster_enitech;

ThrusterEnitechNode::ThrusterEnitechNode(const std::string& name)
    : LifecycleNode(name)
{
    this->declare_parameter<std::string>("thruster_name", ""); // thruster name
    this->declare_parameter<int>("thruster_id", -1); // can id of the device
    this->declare_parameter<double>("update_period", 0.1); // update period of the thruster state, default 0.1 s
    this->declare_parameter<double>("heartbeat_period", 2.0); // heartbeat period, default 2 s
    this->declare_parameter<double>("status_timeout", 5.0); // time without thruster status, default 5 s
}

ThrusterEnitechNode::~ThrusterEnitechNode()
{
    timer_update_.reset();    
    timer_status_timeout_.reset();
    NMT::Stop stop(m_protocol);
    process_request(stop, rclcpp::Duration(std::chrono::duration<double>(0.1)));
    RCLCPP_INFO(this->get_logger(), "Thruster stopped on destructor." );
}   

int ThrusterEnitechNode::get_thruster_by_name(std::vector<std::string> const &names)
{
    for (size_t i = 0; i < names.size(); ++i)
        if (names[i] == thruster_name_)
            return i;
    return -1;
}


void ThrusterEnitechNode::thruster_command_callback(const auv_control_msgs::msg::JointCommandStamped::SharedPtr thruster_command_msg)
{
    int thruster_idx = get_thruster_by_name(thruster_command_msg->names);
    if(thruster_idx==-1)
    {
        RCLCPP_ERROR(this->get_logger(), ("Thruster with name " + thruster_name_ + " is not given in the input command").c_str());
        thruster_cmd_.names.resize(0);
        thruster_cmd_.velocities.resize(0);
        return;
    }
    else
    {
        thruster_cmd_.header = thruster_command_msg->header;
        thruster_cmd_.names.resize(1);
        thruster_cmd_.velocities.resize(1);
        thruster_cmd_.names[0] = thruster_command_msg->names[thruster_idx];
        thruster_cmd_.velocities[0] = thruster_command_msg->velocities[thruster_idx];
        new_thruster_cmd_ = true;
    }
    
}

void ThrusterEnitechNode::can_in_callback(const ros2_canbus::msg::CanbusMessage::SharedPtr can_in_msg)
{
    can_in_ = canbus::rosCanbusMsg2canbusMessage(*can_in_msg);
    new_can_in_ = true;
    //RCLCPP_INFO_STREAM(this->get_logger(), "CAN ID froom callback " << can_in_msg->can_id);

}

bool ThrusterEnitechNode::process_request(Request &request, const rclcpp::Duration timeout)
{
    ros2_canbus::msg::CanbusMessage request_msg;
    request_msg = canbusMessage2rosCanbusMsg(request.message);
    can_publisher_->publish(request_msg);

    canbus::Message prev_can_in = can_in_;
    rclcpp::Time start = this->now();
    rclcpp::sleep_for(100ms);

    do
    {   
        if (can_in_.time > prev_can_in.time)
        {

            if (request.update(can_in_))
                return true;
            prev_can_in = can_in_;
        }
        rclcpp::sleep_for(100ms);
    } while (this->now() - start < timeout);

    return false;
}

void ThrusterEnitechNode::timer_status_timeout_callback()
{
    RCLCPP_ERROR(this->get_logger(), ("Thruster Timeout: " + thruster_name_).c_str());
}

void ThrusterEnitechNode::update_callback()
{
    if(new_can_in_)
    {
        MSG_TYPE msg_type = m_protocol.update(can_in_);
        //if (msg_type == MSG_NONE)
        //    continue;
        if (msg_type == MSG_STATUS)
        {
            //RCLCPP_INFO(this->get_logger(), "Got status Message " );
            timer_status_timeout_->reset();

            Status status = m_protocol.getLastStatus();
            thruster_enitech::msg::ThrusterStatus thruster_status_out;
            thruster_status_out.header.stamp = rclcpp::Time(status.time.time_since_epoch().count());
            thruster_status_out.name = thruster_name_;
            thruster_status_out.speed = status.speed;
            thruster_status_out.current = status.current;
            thruster_status_out.overtemp_bg149 = status.overtemp_bg149;
            thruster_status_out.start_gain = status.start_gain;
            thruster_status_out.air_parameters = status.air_parameters;
            if(status.control_mode==thruster_enitech::Status::SPEED)
                thruster_status_out.control_mode = "speed";
            else if(status.control_mode==thruster_enitech::Status::RAW)
                thruster_status_out.control_mode = "current";

            thruster_state_publisher_->publish(thruster_status_out);
        }
        else if (msg_type == MSG_EMERGENCY)
        {
            Emergency emergency = m_protocol.getLastEmergency();
            RCLCPP_INFO(this->get_logger(), "Got Emergency Message " );
            //_emergency.write(emergency);
        }
        new_can_in_ = false;
    }
    // TODO read temperature

    // create can msg from command and publish
    if(new_thruster_cmd_)
    {
        canbus::Message can_cmd = m_protocol.makeCommand(thruster_cmd_.velocities[0]);
        ros2_canbus::msg::CanbusMessage can_out = canbusMessage2rosCanbusMsg(can_cmd);
        can_publisher_->publish(can_out);
        new_thruster_cmd_ = false;
    }

    std::chrono::system_clock::time_point lastHeartbeat = m_protocol.getLastHeartbeat();
    //_heartbeats.write(lastHeartbeat);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ThrusterEnitechNode::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    // get parameters
    this->get_parameter("thruster_name", thruster_name_);
    this->get_parameter("thruster_id", thruster_id_);
    if(thruster_id_ == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "The thruster ID wasn't set.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // can publisher
    can_publisher_ = this->create_publisher<ros2_canbus::msg::CanbusMessage>(thruster_name_+"/can_out", 10);

    // can subscriber
    // create callback group to make the can subscriber in a parallel thread
    cb_group_can = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = cb_group_can;
    can_subscriber_ = this->create_subscription<ros2_canbus::msg::CanbusMessage>(
        thruster_name_+"/can_in", 10,
        std::bind(&ThrusterEnitechNode::can_in_callback, this, std::placeholders::_1),
        subscription_options);
    // can publisher has to be activated as other configs depend on it
    can_publisher_->on_activate();

    RCLCPP_INFO(this->get_logger(), "CAN publisher and Subscriber established.");

    // Update Callback timer
    std::chrono::milliseconds update_period(10);
    // auto timer_options = rclcpp::TimerOptions();
    // timer_options.callback_group = cb_group_can;
    timer_update_ = this->create_wall_timer(update_period, std::bind(&ThrusterEnitechNode::update_callback, this), cb_group_can);
    timer_update_->cancel();

    // status timeout timer
    double status_timeout_;
    this->get_parameter("status_timeout", status_timeout_);
    timer_status_timeout_ = this->create_wall_timer(std::chrono::duration<double>(status_timeout_), std::bind(&ThrusterEnitechNode::timer_status_timeout_callback, this), cb_group_can);
    timer_status_timeout_->cancel();
    
    thruster_state_publisher_ = this->create_publisher<thruster_enitech::msg::ThrusterStatus>(thruster_name_+"/thruster_status", 10);

    // initialize protocol
    m_protocol = Protocol(thruster_id_);
    RCLCPP_INFO(this->get_logger(), "Protocol instantiated.");
    double heartbeat_period_;
    this->get_parameter("heartbeat_period", heartbeat_period_);
    SDO::WriteHeartbeatPeriod heartbeat(m_protocol, std::chrono::milliseconds(int(heartbeat_period_*1e3)));
 
    rclcpp::Duration hb_timeout(std::chrono::duration<double>(0.5)); // 0.5 sec

    // send heartbeat request and wait for success
    if(!process_request(heartbeat, hb_timeout))
    {
	    RCLCPP_ERROR(this->get_logger(), "Failed to request thruster heartbeat.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(this->get_logger(), "Heartbeat successfully sent" );

    NMT::Reset reset(m_protocol);
    if(!process_request(reset, rclcpp::Duration(std::chrono::duration<double>(2.0))))
    {
	RCLCPP_ERROR(this->get_logger(), "Failed to reset thruster.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(this->get_logger(), "Thruster reset successful." );

    double update_period_;
    this->get_parameter("update_period", update_period_);
    SDO::WriteUpdatePeriod update(m_protocol, std::chrono::milliseconds(int(update_period_*1e3)));
    if(!process_request(update, rclcpp::Duration(std::chrono::duration<double>(0.5))))
    {
	RCLCPP_ERROR(this->get_logger(), "Failed to set thruster update period.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(this->get_logger(), "Thruster update period successfully set." );

    // initialize thruster command subscriber
    thruster_command_subscriber_ = this->create_subscription<auv_control_msgs::msg::JointCommandStamped>(
        thruster_name_+"/CommandWrenchStamped", 10,
        std::bind(&ThrusterEnitechNode::thruster_command_callback, this, std::placeholders::_1));

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ThrusterEnitechNode::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    m_protocol.reset();
    thruster_state_publisher_.reset();
    can_publisher_->on_deactivate();
    can_publisher_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ThrusterEnitechNode::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    thruster_state_publisher_->on_activate();
    NMT::Start start(m_protocol);
    if(!process_request(start, rclcpp::Duration(std::chrono::duration<double>(1.0))))
    {
	    RCLCPP_ERROR(this->get_logger(), "Failed to start thruster.");
        //return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(this->get_logger(), "Thruster started.");


    timer_update_->reset();    
    timer_status_timeout_->reset();
    RCLCPP_INFO(this->get_logger(), "Start update Loop." );
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ThrusterEnitechNode::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    timer_update_.reset();    
    timer_status_timeout_.reset();
    NMT::Stop stop(m_protocol);
    process_request(stop, rclcpp::Duration(std::chrono::duration<double>(0.1)));
    RCLCPP_INFO(this->get_logger(), "Thruster stopped on deactivate." );
    thruster_state_publisher_->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ThrusterEnitechNode::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    timer_update_.reset();    
    timer_status_timeout_.reset();
    NMT::Stop stop(m_protocol);
    process_request(stop, rclcpp::Duration(std::chrono::duration<double>(0.1)));

    RCLCPP_INFO(this->get_logger(), "Thruster stopped shutdown." );
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<ThrusterEnitechNode> node = std::make_shared<ThrusterEnitechNode>("thruster_enitech_node");
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
