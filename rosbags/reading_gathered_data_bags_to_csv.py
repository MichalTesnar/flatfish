import rosbag_api as bag
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

name = 'rosbag2_long_gathered'

bag_file = f'{name}/{name}.db3'
topic_name = '/gathered_data'

### connect to the database
conn, c = bag.connect(bag_file)

### get all topics names and types
topic_names = bag.getAllTopicsNames(c, print_out=False)
topic_types = bag.getAllMsgsTypes(c, print_out=False)

# Create a map for quicker lookup
type_map = {topic_names[i]:topic_types[i] for i in range(len(topic_types))}

### get all timestamps and all messages
t, msgs = bag.getAllMessagesInTopic(c, topic_name, print_out=False)

import csv

# Open a CSV file in write mode
with open(f'{name}.csv', 'w', newline='') as csvfile:
    # Create a CSV writer object
    csv_writer = csv.writer(csvfile)
    
    # Write header row
    csv_writer.writerow(['time', 'linear_x', 'linear_y', 'angular_z', 'thruster_surge_left', 'thruster_surge_right', 'thruster_sway_front', 'thruster_sway_rear'])


    # Loop through each message
    msg_type = get_message(type_map[topic_name])
    for i in range(len(msgs)):
        mess = deserialize_message(msgs[i], msg_type)
        time = mess.header.stamp.sec + mess.header.stamp.nanosec/1e9

        csv_writer.writerow([time, mess.twist.linear.x, mess.twist.linear.y, mess.twist.angular.z, mess.thrusters.speed_surge_left, mess.thrusters.speed_surge_right, mess.thrusters.speed_sway_front, mess.thrusters.speed_sway_rear])

# Close connection to the database
bag.close(conn)