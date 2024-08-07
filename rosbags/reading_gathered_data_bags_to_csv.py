import rosbag_api as bag
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

import sys

# name = 'long_mission_for_test_set'
name = sys.argv[1]

bag_file = f'{name}/{name}_0.db3'
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

    # Loop through each message
    msg_type = get_message(type_map[topic_name])
    for i in range(len(msgs)):
        mess = deserialize_message(msgs[i], msg_type)
        time = mess.header.stamp.sec + mess.header.stamp.nanosec/1e9

        csv_writer.writerow([mess.sample[0], mess.sample[1], mess.sample[2], mess.sample[3], mess.sample[4], mess.sample[5], mess.target[0], mess.target[1], mess.target[2], mess.target[3]])

# Close connection to the database
bag.close(conn)