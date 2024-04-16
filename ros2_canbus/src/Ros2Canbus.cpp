/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include <Ros2Canbus.hpp>


using namespace canbus;
using namespace ros2_canbus;
using namespace std::chrono_literals;

Ros2CanbusNode::Ros2CanbusNode(std::string const& name, bool intra_process_comms)
: rclcpp_lifecycle::LifecycleNode(name,
  rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
, m_driver(NULL)
{
    m_node_name = this->get_name();
    m_declareRosParameters();
    RCLCPP_INFO_STREAM(get_logger(), "The lifecycle node " << m_node_name << " is running");
}

Ros2CanbusNode::~Ros2CanbusNode()
{
    if (this->get_current_state().label() != "unconfigured") {
        m_unsetRosNode();
    }
    m_closeCanPort();
    RCLCPP_WARN_STREAM(this->get_logger(), "Destroying the node " << m_node_name);
}

void Ros2CanbusNode::m_setRosNode()
{
    m_createTimers();
    m_createPublishers();
    m_createSubscriptions();
    m_createServices();
}

void Ros2CanbusNode::m_unsetRosNode()
{
    m_deleteTimers();
    m_deletePublishers();
    m_deleteSubscriptions();
    m_deleteServices();
}

void Ros2CanbusNode::m_activateRosNode()
{
    // activate timers for constant update
    m_activateTimers();
    // Activate publishers
    m_activatePublishers();
    // Activate subsribers
    // NOTE: Still not implemented in lifecycle
    m_activateSubscriptions();
    // Activate subsribers
    // NOTE: Still not implemented in lifecycle
    m_activateServices();
}

void Ros2CanbusNode::m_deactivateRosNode()
{
    // Stop timers
    m_deactivateTimers();
    // Deactivate publishers
    m_deactivatePublishers();
    // Deactivate subscriptions
    // NOTE: Still not implemented in lifecycle
    m_deactivateSubscriptions();
    // Deactivate subscriptions
    // NOTE: Still not implemented in lifecycle
    m_deactivateServices();
}

void Ros2CanbusNode::m_declareRosParameters()
{
    // TODO test
    //
    // auto param_listener = std::make_shared<canbus::ParamListener>(this->get_node_parameters_interface());
    // auto params = param_listener->get_params();

    // the device file used to connect to the CAN bus
    this->declare_parameter<std::string>("device", "can1");
    // TDOO check default

    // the device type, this determines which driver will be used to open the device
    // SOCKET       = 0
    // HICO         = 1
    // HICO_PCI     = 2
    // VS_CAN       = 3
    // CAN2WEB      = 4
    // NET_GATEWAY  = 5
    // EASY_SYNC    = 6
    this->declare_parameter<uint8_t>("device_type", 0);

    // this property defines the interval in which the can bus is checked for correct function.
    // The unit is in s
    this->declare_parameter<double>("check_bus_ok_interval", 0.1);

    // the interval in which the stats message is send. The unit is in s
    this->declare_parameter<double>("stats_interval", 1.0);

    // frequency that the update function runs. The unit is in Hz
    this->declare_parameter<double>("system_frequency", 100.0);

    // watch devices
    // array of ids
    // this->declare_parameter("watch_devices.can_id");
    this->declare_parameter<std::vector<int64_t>>("watch_devices.can_id", std::vector<int64_t>());
    // array of names
    // this->declare_parameter("watch_devices.name");
    this->declare_parameter<std::vector<std::string>>("watch_devices.name", std::vector<std::string>());

}

void Ros2CanbusNode::m_createSubscriptions()
{
    m_sub_tx_message = this->create_subscription<ros2_canbus::msg::CanbusMessage>(
      (m_node_name + "/tx_message").c_str(), QOS_HISTORY_DEPTH,
      std::bind(&Ros2CanbusNode::cb_txMessage, this, std::placeholders::_1));

    m_sub_tx_messages = this->create_subscription<ros2_canbus::msg::ArrayCanbusMessage>(
      (m_node_name + "/tx_messages").c_str(), QOS_HISTORY_DEPTH,
      std::bind(&Ros2CanbusNode::cb_txMessages, this, std::placeholders::_1));

    // TODO: do I need this?
    // Notification event topic. All state changes
    // are published here as TransitionEvents with
    // a start and goal state indicating the transition
    // m_sub_notification = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
    //   "transition_event",
    //   QOS_HISTORY_DEPTH,
    //   std::bind(&Ros2CanbusNode::cb_notification, this, std::placeholders::_1));
}

void Ros2CanbusNode::m_activateSubscriptions()
{
    // NOTE: still not program by lifecycleNode
    // TODO: uncomment when implemeneted
    // m_sub_tx_message->on_activate();
    // m_sub_tx_messages->on_activate();
    // m_sub_notification->on_activate();
}

void Ros2CanbusNode::m_deactivateSubscriptions()
{
    // NOTE: still not program by lifecycleNode
    // TODO: uncomment when implemeneted
    // m_sub_tx_message->on_deactivate();
    // m_sub_tx_messages->on_deactivate();
    // m_sub_notification->on_deactivate();
}

void Ros2CanbusNode::m_deleteSubscriptions()
{
    m_deactivateSubscriptions();
    // Delete subscribers
    m_sub_tx_message.reset();
    m_sub_tx_messages.reset();
    // m_sub_notification.reset();
}

void Ros2CanbusNode::m_createPublishers()
{
    // Create publishers
    // NOTE: the publisher for the watched devices are created while inserting a new watched device
    m_pub_msgs_from_unwatched_device = this->create_publisher<ros2_canbus::msg::CanbusMessage>((m_node_name + "/rx_unwatched_message").c_str(),QOS_HISTORY_DEPTH);
    m_pub_msgs_from_unwatched_devices = this->create_publisher<ros2_canbus::msg::ArrayCanbusMessage>((m_node_name + "/rx_unwatched_messages").c_str(),QOS_HISTORY_DEPTH);
    m_pub_statistics = this->create_publisher<ros2_canbus::msg::CanbusStatistics>((m_node_name + "/statistics").c_str(),QOS_HISTORY_DEPTH);
}

void Ros2CanbusNode::m_activatePublishers()
{
    m_activatePublishersOfWatchedDevices();
    m_pub_msgs_from_unwatched_device->on_activate();
    m_pub_msgs_from_unwatched_devices->on_activate();
    m_pub_statistics->on_activate();
}

void Ros2CanbusNode::m_deactivatePublishers()
{
    // NOTE: still not program by lifecycleNode
    m_deactivatePublishersOfWatchedDevices();
    m_pub_msgs_from_unwatched_device->on_deactivate();
    m_pub_msgs_from_unwatched_devices->on_deactivate();
    m_pub_statistics->on_deactivate();
}


void Ros2CanbusNode::m_deletePublishers()
{
    // Delete publishers
    m_deactivatePublishers();
    m_unwatchAll();
    m_pub_msgs_from_unwatched_device.reset();
    m_pub_msgs_from_unwatched_devices.reset();
    m_pub_statistics.reset();
}


void Ros2CanbusNode::m_createServices()
{
    m_srv_watch = this->create_service<ros2_canbus::srv::Watch>((m_node_name + "/watch").c_str(),
      std::bind(&Ros2CanbusNode::cb_watch, this, std::placeholders::_1,  std::placeholders::_2));

    m_srv_unwatch = this->create_service<ros2_canbus::srv::Unwatch>((m_node_name + "/unwatch").c_str(),
      std::bind(&Ros2CanbusNode::cb_unwatch, this, std::placeholders::_1,  std::placeholders::_2));

    m_srv_get_topic_name = this->create_service<ros2_canbus::srv::GetTopicName>((m_node_name + "/get_topic_name").c_str(),
      std::bind(&Ros2CanbusNode::cb_get_topic_name, this, std::placeholders::_1,  std::placeholders::_2));

    m_srv_is_watched = this->create_service<ros2_canbus::srv::IsWatched>((m_node_name + "/is_watched").c_str(),
      std::bind(&Ros2CanbusNode::cb_is_watched, this, std::placeholders::_1,  std::placeholders::_2));
}

void Ros2CanbusNode::m_activateServices()
{
    // NOTE: still not program by lifecycleNode
    // TODO: uncomment when implemeneted
    // m_srv_watch->on_activate();
    // m_srv_unwatch->on_activate();
    // m_srv_get_topic_name->on_activate();
    // m_srv_is_watched->on_activate();
}

void Ros2CanbusNode::m_deactivateServices()
{
    // NOTE: still not program by lifecycleNode
    // TODO: uncomment when implemeneted
    // m_srv_watch->on_deactivate();
    // m_srv_unwatch->on_deactivate();
    // m_srv_get_topic_name->on_deactivate();
    // m_srv_is_watched->on_deactivate();
}

void Ros2CanbusNode::m_deleteServices()
{
    m_deactivateServices();
    m_srv_watch.reset();
    m_srv_unwatch.reset();
    m_srv_get_topic_name.reset();
    m_srv_is_watched.reset();
}


void Ros2CanbusNode::m_activatePublishersOfWatchedDevices()
{
    for (DeviceMapping& watched_device : m_watched_devices)
    {
        watched_device.publisher->on_activate();
    }
}

void Ros2CanbusNode::m_deactivatePublishersOfWatchedDevices()
{
    for (DeviceMapping& watched_device : m_watched_devices)
    {
        watched_device.publisher->on_deactivate();
    }
}


void Ros2CanbusNode::m_createTimers()
{
    rclcpp::Parameter param_system_frequency;
    rclcpp::Parameter param_check_bus_ok_interval;
    rclcpp::Parameter param_stats_interval;

    this->get_parameter("system_frequency", param_system_frequency);
    this->get_parameter("check_bus_ok_interval", param_check_bus_ok_interval);
    this->get_parameter("stats_interval", param_stats_interval);

    const double system_frequency = param_system_frequency.as_double();
    const double check_bus_ok_interval = param_check_bus_ok_interval.as_double();
    const double stats_interval = param_stats_interval.as_double();

    // NOTE: ros Timer requires a chrono duration and not ros duration :/
    // m_check_bus_ok_interval = ros_utils::durationInSeconds2rosDuration<double>(param_checkBusOkInterval);
    // m_stats_interval = ros_utils::durationInSeconds2rosDuration<double>(param_statsInterval);

    // setup timers for constant update
    m_update_timer = this->create_wall_timer(
      std::chrono::duration<double>(1/system_frequency),
      std::bind(&Ros2CanbusNode::on_update, this));

    m_check_bus_ok_timer = this->create_wall_timer(
      std::chrono::duration<double>(check_bus_ok_interval),
      std::bind(&Ros2CanbusNode::on_checkBusOk, this));

    m_publish_statistics_timer = this->create_wall_timer(
      std::chrono::duration<double>(stats_interval),
      std::bind(&Ros2CanbusNode::on_publishStatistics, this));

    // when create the timers start ticking
    // by cancel them they stop til reset
    m_deactivateTimers();
}

void Ros2CanbusNode::m_activateTimers()
{
    m_update_timer->reset();
    m_check_bus_ok_timer->reset();
    m_publish_statistics_timer->reset();
}

void Ros2CanbusNode::m_deactivateTimers()
{
    m_update_timer->cancel();
    m_check_bus_ok_timer->cancel();
    m_publish_statistics_timer->cancel();
}

void Ros2CanbusNode::m_deleteTimers()
{
    // delete timers
    m_deactivateTimers();
    // NOTE: .reset the share pointer, i.e. ownership
    m_update_timer.reset();
    m_check_bus_ok_timer.reset();
    m_publish_statistics_timer.reset();
}

bool Ros2CanbusNode::m_watch(std::string const& name, uint32_t id)
{
    // check whether id or name is already used
    if (m_isWatched(id) && m_isWatched(name)){
        RCLCPP_WARN_STREAM(get_logger(), "The CAN device with id " << id << " cannot be watched");
        return false;
    }
    else {
        // create new device
        DeviceMapping device;
        device.name = name;
        device.id = id;
        // device.mask = mask;
        device.publisher = this->create_publisher<ros2_canbus::msg::CanbusMessage>((m_node_name + '/' + device.name + "/output").c_str(), QOS_HISTORY_DEPTH);

        // NOTE: bug in ros2 see: https://answers.ros.org/question/308386/ros2-add-arguments-to-callback/ 
        std::function<void(const ros2_canbus::msg::CanbusMessage::SharedPtr msg)> fcn = std::bind(&Ros2CanbusNode::cb_txMessageFromWatched, this, std::placeholders::_1, device.id);
        device.subscription = this->create_subscription<ros2_canbus::msg::CanbusMessage>(
            (m_node_name + '/' + device.name + "/input").c_str(), QOS_HISTORY_DEPTH,
            fcn);
            // std::bind(&Ros2CanbusNode::cb_txMessageFromWatched, this, std::placeholders::_1, device.id));

        //check whether the state is active
        if (this->get_current_state().label() == "active") {
            device.publisher->on_activate();
        }
        m_watched_devices.push_back(device);

        // update global mask
        m_global_mask |= device.id;
        RCLCPP_INFO_STREAM(get_logger(), "The CAN device "<< device.name <<  "with id " << id << " is watched now");
        return true;
    }
}

void Ros2CanbusNode::m_unwatchAll()
{
    // unwatch all devices
    for (Ros2CanbusNode::DeviceMappings::iterator it = m_watched_devices.begin(); it != m_watched_devices.end();)
    {
        it = m_unwatch(it);
    }
}

Ros2CanbusNode::DeviceMappings::iterator Ros2CanbusNode::m_unwatch(Ros2CanbusNode::DeviceMappings::iterator watched_device_it){
    // NOTE: first disactivate the publisher then release ownership
    watched_device_it->publisher->on_deactivate();
    watched_device_it->publisher.reset();
    //NOTE: still no deactivate for subscriptions
    // watched_device_it->subscription->on_deactivate();
    watched_device_it->subscription.reset();
    RCLCPP_INFO_STREAM(get_logger(), "The CAN device with id " << watched_device_it->id << " is unwatched now");
    return m_watched_devices.erase(watched_device_it);
}


bool Ros2CanbusNode::m_unwatch(const uint32_t id)
{
    // Erase (all) elements with this id (in case multiple publishers with one id (?))
    // NOTE: there is only a unique device with an id as there is check function,
    for (Ros2CanbusNode::DeviceMappings::iterator it = m_watched_devices.begin(); it != m_watched_devices.end(); ++it)
    {
        if (it->id == id){
            m_unwatch(it);
            return true;
        }
    }
    return false;
}

// bool Ros2CanbusNode::m_unwatch(std::string const& name)
// {
//     // Erase (all) elements with this id (in case multiple publishers with one id (?))
//     // NOTE: there is only a unique device with an id as there is check function,
//     for (Ros2CanbusNode::DeviceMappings::iterator it = m_watched_devices.begin(); it != m_watched_devices.end();; ++it)
//     {
//         if (it->name == name)
//             m_unwatch(it);
//             return true;
//     }
//     return false;
// }

int Ros2CanbusNode::m_findWatchedDevice(const uint32_t id)
{
    // TODO: use mask (make that sense?)
    // NOTE: I don't see why that should save time with individual masks (?)
    // global mask will help though ()
    if (m_isWatched(id)){
        for (size_t i = 0; i < m_watched_devices.size(); i++){
            if (m_watched_devices[i].id == id){
                return i;
            }
        }
    }
    else {
        return -1;
    }
}

// TODO do we need this! I have to figure out something else!
bool Ros2CanbusNode::m_isWatched(std::string const& name)
{
    for (DeviceMapping& watched_device : m_watched_devices)
    {
        if (watched_device.name == name){
            RCLCPP_INFO_STREAM(get_logger(),
                "The specified name device " << name <<
                "is already in use with the CAN ID" << watched_device.id);
            return true;
        }
    }
    return false;
}

bool Ros2CanbusNode::m_isWatched(const uint32_t id)
{
    // using global mask
    return ((id & m_global_mask) == id);

    // for (DeviceMapping& watched_device : m_watched_devices)
    // {
    //     if (watched_device.id == id){
    //         RCLCPP_INFO_STREAM(get_logger(),
    //             "The specified CAN device with ID" << id <<
    //             "is already watched and publishing as " << watched_device.name);
    //         return true;
    //     }
    // }
    // return false;
}

bool Ros2CanbusNode::m_openCanPort(std::string const& device, const canbus::DRIVER_TYPE device_type)
{
    return (m_driver = canbus::openCanDevice(device, device_type));
}

bool Ros2CanbusNode::m_resetCanPort()
{
    if (!m_driver->reset())
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "CANBUS: Failed to reset can driver");
        return false;
    }
    m_driver->clear(); //TODO: check this if it's right
    return true;
}

void Ros2CanbusNode::m_closeCanPort()
{
    // close can port
    m_driver->close();
    delete m_driver;
    m_driver = 0;
}

void Ros2CanbusNode::m_readCanbus()
{
    // Read the data on the file descriptor (if there is any) and push it on the
    // matching port. We ask the board how many packets there is to read.
    int msg_count = m_driver->getPendingMessagesCount();

    // in case of messages, we reset the timeout
    if (msg_count > 0)
    {
        m_check_bus_ok_timer.reset();
    }

    // check the messages
    for (int i = 0; i < msg_count; ++i)
    {
        auto msg = m_driver->read();
        RCLCPP_INFO_STREAM(get_logger(), "Get new message with id " << msg.can_id << " to be transmitted");

        m_stats.msg_rx++;
        // CAN extended frames are 8 bytes of header, max 8 bytes of payload
        m_stats.rx += 8 + msg.size;

        // check whether the can message belongs to the watch ones
        auto index = m_findWatchedDevice(msg.can_id);


        // message from unwatched device
        if (index == -1){
            RCLCPP_INFO_STREAM(get_logger(), "Not found");
            m_unwatched_devices.data.push_back(canbus::canbusMessage2rosCanbusMsg(msg));
        }
        // message from watched device
        else {
            RCLCPP_INFO_STREAM(get_logger(), "Found");
            m_watched_devices[index].message = canbus::canbusMessage2rosCanbusMsg(msg);
            m_watched_devices[index].newMessage = true;
        }
    }
}


void Ros2CanbusNode::m_writeCanbus()
{
    // Write the data that is available on the input ports
    while (!m_queue_tx_messages.empty())
    {
        m_stats.msg_tx++;
        // CAN extended frames are 8 bytes of header, max 8 bytes of payload
        m_stats.tx += 8 + m_queue_tx_messages.front().size;
        m_driver->write(m_queue_tx_messages.front());
        m_queue_tx_messages.pop();
    }
}

void Ros2CanbusNode::m_publishCanbusMsgsFromWatchedDevices()
{
    for (DeviceMapping& watched_device : m_watched_devices)
    {
        // as there are watched they are always published as latched
        // NOTE: latching is made in the subscriber and not the publisher: https://answers.ros.org/question/305795/ros2-latching/
        if (watched_device.newMessage){
            watched_device.publisher->publish(watched_device.message);
            watched_device.newMessage = false;
        }
    }
}


void Ros2CanbusNode::m_publishCanbusMsgsFromUnwatchedDevices()
{
    // only if subscribers and data
    const bool publish_can_msgs = (m_pub_msgs_from_unwatched_device->get_subscription_count() > 0) & (m_unwatched_devices.data.size() > 0);
    if (publish_can_msgs)
    {
        for (auto& msg_unwatched_device : m_unwatched_devices.data){
            m_pub_msgs_from_unwatched_device->publish(msg_unwatched_device);
        }
    }

    const bool publish_can_msgs_as_array = (m_pub_msgs_from_unwatched_devices->get_subscription_count() > 0) & (m_unwatched_devices.data.size() > 0);
    if (publish_can_msgs_as_array)
    {
        m_pub_msgs_from_unwatched_devices->publish(m_unwatched_devices);
    }

    // clear message independent of whether they have been published
    m_unwatched_devices.data.clear();
}


void Ros2CanbusNode::m_publishCanbusStatistics()
{
    // only if subscribers
    const bool publish_statistics = (m_pub_statistics->get_subscription_count() > 0);
    if (publish_statistics)
    {
        // update stamp
        // TODO difference between both
        // m_stats.stamp = this->get_clock()->now().to_msg();
        m_stats.stamp = this->now();
        m_stats.error_count =  m_driver->getErrorCount();
        m_pub_statistics->publish(m_stats);
    }
}

Ros2CanbusNode::Lifecycle::CallbackReturn Ros2CanbusNode::on_configure(const rclcpp_lifecycle::State &previous_state)
{


    // get parameters to open can
    rclcpp::Parameter param_device;
    rclcpp::Parameter param_deviceType;

    this->get_parameter("device", param_device);
    this->get_parameter("device_type", param_deviceType);

    std::string device = param_device.as_string();
    uint8_t deviceType = param_deviceType.as_int();

    // open can
    if (!m_openCanPort(device, static_cast<canbus::DRIVER_TYPE>(deviceType))){
        RCLCPP_ERROR_STREAM(this->get_logger(), "CANBUS: Failed to open device");
        return Ros2CanbusNode::Lifecycle::CallbackReturn::FAILURE;
    }

    // if can open
    // setup ros
    m_setRosNode();

    // get parameters for devices
    rclcpp::Parameter param_watch_devices_can_id;
    rclcpp::Parameter param_watch_devices_name;

    this->get_parameter("watch_devices.can_id", param_watch_devices_can_id);
    this->get_parameter("watch_devices.name", param_watch_devices_name);

    std::vector<int64_t> watch_devices_can_id = (param_watch_devices_can_id.as_integer_array());
    std::vector<std::string> watch_devices_name = param_watch_devices_name.as_string_array();

    // assert
    // TODO: customize assert (?)
    assert(watch_devices_can_id.size() == watch_devices_name.size() && "The size of can ids and names should be equal");

    for (size_t i = 0; i < watch_devices_can_id.size(); i++)
    {
        m_watch(watch_devices_name[i], watch_devices_can_id[i]);
    }

    return Ros2CanbusNode::Lifecycle::CallbackReturn::SUCCESS;
}

Ros2CanbusNode::Lifecycle::CallbackReturn Ros2CanbusNode::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    m_unsetRosNode();
    m_closeCanPort();
    return Ros2CanbusNode::Lifecycle::CallbackReturn::SUCCESS;
}

Ros2CanbusNode::Lifecycle::CallbackReturn Ros2CanbusNode::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    // reset canbus
    if (!m_resetCanPort()){
        Ros2CanbusNode::Lifecycle::CallbackReturn::FAILURE;
    }

    m_activateRosNode();

    return Ros2CanbusNode::Lifecycle::CallbackReturn::SUCCESS;
}

Ros2CanbusNode::Lifecycle::CallbackReturn Ros2CanbusNode::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    // publish the remaining messages before deactivating
    m_publishCanbusMsgsFromWatchedDevices();
    m_publishCanbusMsgsFromUnwatchedDevices();

    m_deactivateRosNode();

    return Ros2CanbusNode::Lifecycle::CallbackReturn::SUCCESS;
}


Ros2CanbusNode::Lifecycle::CallbackReturn Ros2CanbusNode::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    m_unsetRosNode();
    m_closeCanPort();

    RCLCPP_WARN_STREAM(this->get_logger(), "Shutting down the node " << m_node_name);
    // NOTE: the lifecycle still holds although one cannot restart the node?
    // delete this;
    return Ros2CanbusNode::Lifecycle::CallbackReturn::SUCCESS;
}


void Ros2CanbusNode::on_update()
{
    m_readCanbus();
    m_publishCanbusMsgsFromWatchedDevices();
    m_publishCanbusMsgsFromUnwatchedDevices();
    m_writeCanbus();
}


void Ros2CanbusNode::on_checkBusOk()
{
    if(!m_driver->checkBusOk()) {
        RCLCPP_WARN_STREAM(this->get_logger(), "CANBUS: reported error");
    }
}


void Ros2CanbusNode::on_publishStatistics()
{
    m_publishCanbusStatistics();
}

// m_nh.subscribe<level1_msgs::RelativeFlickerLocation>(GAC_FLICKER_TOPIC_NAME, QUEUE_SIZE, boost::bind(&Master::cb_rxNewFlickerData, this, _1, GAC_FLICKER_TOPIC_NAME))

void Ros2CanbusNode::cb_txMessageFromWatched(ros2_canbus::msg::CanbusMessage::ConstSharedPtr msg, const uint32_t can_id)
{
    // TODO: as activate and deactivate subscription is still not done
    // we have to check whether we are in the "active" state
    if (this->get_current_state().label() == "active"){
        auto tx_message = canbus::rosCanbusMsg2canbusMessage(*msg);
        tx_message.can_id = can_id;
        m_queue_tx_messages.push(tx_message);
        RCLCPP_INFO_STREAM(get_logger(), "Get new message from watched device with id " << tx_message.can_id << " to be transmitted");
    }
}

void Ros2CanbusNode::cb_txMessage(ros2_canbus::msg::CanbusMessage::ConstSharedPtr msg)
{
    // TODO: as activate and deactivate subscription is still not done
    // we have to check whether we are in the "active" state
    if (this->get_current_state().label() == "active"){
        m_queue_tx_messages.push(canbus::rosCanbusMsg2canbusMessage(*msg));
        RCLCPP_INFO_STREAM(get_logger(), "Get new message with id " << msg->can_id << " to be transmitted");
    }
}

void Ros2CanbusNode::cb_txMessages(ros2_canbus::msg::ArrayCanbusMessage::ConstSharedPtr msg)
{
    // TODO: as activate and deactivate subscription is still not done
    // we have to check whether we are in the "active" state
    if (this->get_current_state().label() == "active"){
        // we transform the input to canbus messages on the fly to save time in the update loop
        // std::queue does not have this option
        // m_queue_tx_messages.insert(m_queue_tx_messages.end(), msg.data.begin(),msg.data.end());
        for (size_t i = 0; i < msg->data.size(); i++)
        {
            m_queue_tx_messages.push(canbus::rosCanbusMsg2canbusMessage(msg->data[i]));
        }
        
        // for (ros2_canbus::msg::CanbusMessage& tx_message : msg->data)
        // {
        //     m_queue_tx_messages.push(canbus::rosCanbusMsg2canbusMessage(tx_message));

        // }
        RCLCPP_INFO_STREAM(get_logger(), "Get " << msg->data.size() << " new messages to be transmitted");
    }
}

void Ros2CanbusNode::cb_watch
    (const std::shared_ptr<ros2_canbus::srv::Watch::Request>     request,
     std::shared_ptr<ros2_canbus::srv::Watch::Response>          response)
{
    if (m_watch(request->name, request->can_id)){
        response->result = ros2_canbus::srv::Watch::Response::DONE;
    }
    else {
        if (m_isWatched(request->can_id)){
            response->result = ros2_canbus::srv::Watch::Response::ERROR_EXIST_ID;
        }
        else if (m_isWatched(request->name)){
            response->result = ros2_canbus::srv::Watch::Response::ERROR_EXIST_NAME;
        }
        else {
            response->result = ros2_canbus::srv::Watch::Response::ERROR;
        }
    }
    // if (this->get_current_state().label() == "active" ||
    //     this->get_current_state().label() == "inactive"){
    // }
    // else {
    //     response->result = ros2_canbus::srv::Watch::Response::ERROR;
    // }
}
void Ros2CanbusNode::cb_unwatch
    (const std::shared_ptr<ros2_canbus::srv::Unwatch::Request>   request,
     std::shared_ptr<ros2_canbus::srv::Unwatch::Response>        response)
{
    // TODO
    if (m_unwatch(request->can_id)){
        response->result = ros2_canbus::srv::Unwatch::Response::DONE;
    }
    else if (!m_isWatched(request->can_id)) {
        response->result = ros2_canbus::srv::Unwatch::Response::ERROR_NO_EXIST_ID;
    }
    else {
        response->result = ros2_canbus::srv::Unwatch::Response::ERROR;
    }

    // if (this->get_current_state().label() == "active" ||
    //     this->get_current_state().label() == "inactive"){

    // }
    // else {
    //     response->result = ros2_canbus::srv::Watch::Response::ERROR;
    // }
}
void Ros2CanbusNode::cb_get_topic_name
    (const std::shared_ptr<ros2_canbus::srv::GetTopicName::Request>  request,
     std::shared_ptr<ros2_canbus::srv::GetTopicName::Response>   response)
{
    if (m_isWatched(request->can_id)){
        response->is_watched = true;
        const auto index = m_findWatchedDevice(request->can_id);
        response->topic_name = m_node_name + '/' + m_watched_devices.at(index).name;
    }
    else {
        response->is_watched = false;
    }
    // if (this->get_current_state().label() == "active" ||
    //     this->get_current_state().label() == "inactive"){
    // }
    // else {
    //     response->is_watched = false;
    // }
}
void Ros2CanbusNode::cb_is_watched
    (const std::shared_ptr<ros2_canbus::srv::IsWatched::Request> request,
     std::shared_ptr<ros2_canbus::srv::IsWatched::Response>      response)
{

    response->is_watched = m_isWatched(request->can_id);
    // if (this->get_current_state().label() == "active" ||
    //     this->get_current_state().label() == "inactive"){
    // }
    // else {
    //     response->is_watched = false;
    // }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    std::shared_ptr<Ros2CanbusNode> canbus_node = std::make_shared<Ros2CanbusNode>("canbus");

    executor.add_node(canbus_node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
