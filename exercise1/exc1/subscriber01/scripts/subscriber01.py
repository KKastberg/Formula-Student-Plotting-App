#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt8, Float32

# Params
NODE_NAME = "subscriber01"       # Node name
SUBSCRIBER_TOPIC = "/Kastberg"   # Subscriber topic
PUBLISH_TOPIC = "/kthfs/result"  # Publisher topic
QUEUE_SIZE = 10                  # Publisher queue size
FREQUENCY = 20                   # Send freqency (Hz)
Q = 0.15                         # Input divide value


# Publisher function
# - publishes the input divided by Q at the frequency of FREQUENCY
def publisher_callback(data):
    global pub, accumelated_input, last_input_value
    input_value = data.data

    # Calculate result, with respect to the periodical
    # reset of the input due to the max size of UInt8
    if input_value < last_input_value:
        accumelated_input += last_input_value
    output_result = (input_value + accumelated_input) / Q
    last_input_value = input_value

    # Publish result
    pub.publish(output_result)


# Subscriber function
# - Subscribes to SUBSCRIBER_TOPIC and calls the publisher function
def subscriber():
    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.Subscriber(SUBSCRIBER_TOPIC, UInt8, publisher_callback)

    # Keep the python program running
    rospy.spin()


# Start the node
if __name__ == '__main__':
    # Init publisher
    pub = rospy.Publisher(PUBLISH_TOPIC, Float32, queue_size=QUEUE_SIZE)
    accumelated_input = 0
    last_input_value = 0

    # Start subscriber
    subscriber()
