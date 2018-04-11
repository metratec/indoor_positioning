#!/usr/bin/env python
"""
DESCRIPTION
"""

import rospy
from std_msgs.msg import String


class IPS:
    """
    DESCRIPTION
    """
    def __init__(self):
        # subscribe to raw messages from USB stick
        self.beacon_sub = rospy.Subscriber('ips/receiver/raw', String, self.callback)
        # number of messages to keep
        self.buffer_length = 10
        # list of incoming messages
        self.msg_buffer = []

    def callback(self, msg):
        """
        Append incoming messages to list of previous messages.
        :param msg: String, message of subscribed topic
        """
        # append message to buffer
        self.msg_buffer.append(msg.data)
        # delete oldest message if buffer is full
        if len(self.msg_buffer) > 10:
            del(self.msg_buffer[-1])

    def publish(self):
        pass


if __name__ == '__main__':
    # start node
    rospy.init_node('positioning', anonymous=True)
    # initialize IPSReceiver class
    ips = IPS()
    try:
        # publish receiver messages
        ips.publish()
    except rospy.ROSInterruptException:
        pass
