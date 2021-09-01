#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt8

# Params
NODE_NAME = "publisher01"   # Node name
TOPIC = "/Kastberg"         # Publisher topic
FREQUENCY = 20              # Send freqency (Hz)
QUEUE_SIZE = 10             # Publisher queue size
K = 0                       # Initial msg value
N = 4                       # Increment step size


# Publisher function
# - publishes values, beginning from K, incremented by N at
# - a frequency of FREQUENCY to the topic TOPIC
def publisher():
    msg_value = K

    # Init the subscriber node
    pub = rospy.Publisher(TOPIC, UInt8, queue_size=QUEUE_SIZE)
    rospy.init_node('publisher01', anonymous=True)

    # Set frequency
    rate = rospy.Rate(FREQUENCY)

    # Send the messeges
    while not rospy.is_shutdown():
        pub.publish(msg_value)
        rate.sleep()
        msg_value += N

        # Reset the msg_value to N when reaching max size of UInt8
        msg_value = N if msg_value >= 255 else msg_value


# Start the publisher
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
