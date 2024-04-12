import rosbag_api as bag
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

bag_file = 'rosbag2_short_infered_data/rosbag2_short_infered_data.db3'
topic_name = '/infered_data'

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
with open('data.csv', 'w', newline='') as csvfile:
    # Create a CSV writer object
    csv_writer = csv.writer(csvfile)
    
    # Write header row
    csv_writer.writerow(['linear_x', 'linear_y', 'linear_z', 'thruster_surge_left', 'thruster_surge_right', 'thruster_sway_front', 'thruster_sway_rear', 'accelerations_linear_x', 'accelerations_linear_y',
    'accelerations.angular_z'
    ])


    # Loop through each message
    msg_type = get_message(type_map[topic_name])
    for i in range(len(msgs)):
        mess = deserialize_message(msgs[i], msg_type)
        csv_writer.writerow([mess.sample[0], mess.sample[1], mess.sample[2], mess.sample[3], mess.sample[4], mess.sample[5], mess.sample[6], mess.target[0], mess.target[1], mess.target[2]])

# Close connection to the database
bag.close(conn)