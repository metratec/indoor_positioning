#!/usr/bin/env python
"""
DESCRIPTION
queue_size should probably be at least equal to number of beacons
buffer_length should probably be at least 2x number of beacons
"""

import os
import rospy
from std_msgs.msg import String
from ros_ips.msg import StringStamped
from ros_ips.positioning_plus import PositioningPlus


class IPSplus:
    def __init__(self):
        # publisher for serial messages to be sent to the receiver
        self.receiver_send_pub = rospy.Publisher('ips/receiver/send', String, queue_size=3)
        # subscribe to raw messages of receiver
        self.receiver_sub = rospy.Subscriber('ips/receiver/raw', StringStamped, self.callback)

        # number of messages to keep
        self.bcn_buffer_length = 6  # TODO make configurable
        # list of incoming messages
        self.bcn_buffer = []
        # timestamp from last received message
        self.bcn_last_time = None

        # number of messages to keep
        self.srg_buffer_length = 3  # TODO make configurable
        # list of incoming messages
        self.srg_buffer = []
        # timestamp from last received message
        self.srg_last_time = None
        # flag that is set when a SRG message is received
        self.srg_wait = True

        # initialize positioning class
        config_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config/zones.yml')
        self.positioning = PositioningPlus(config_dir)

        # set publishing rate
        self.rate = rospy.Rate(0.1)

    def callback(self, msg):
        """
        Append incoming messages to list of previous messages.
        :param msg: String, message of subscribed topic
        """
        if msg.data.split(' ')[0] == 'BCN':
            buff_len = self.bcn_buffer_length
            buff = self.bcn_buffer
            self.bcn_last_time = msg.header.stamp
        elif msg.data.split(' ')[0] == 'SRG':
            buff_len = self.srg_buffer_length
            buff = self.srg_buffer
            self.srg_last_time = msg.header.stamp
            self.srg_wait = False
            # TODO handle exceptions like TOE etc.
        else:
            return
        # append message to buffer
        print(msg.data)
        buff.append(msg.data)
        # delete oldest message if buffer is full
        if len(buff) > buff_len:
            del(buff[0])
        # TODO docs

    def publish(self):
        while not rospy.is_shutdown():
            # get three beacons in range with the highest RSSI values
            # beacons = self.positioning.get_top_beacons(self.msg_buffer, 3)  # TODO support more (or less) than 3 beacons
            # get all the beacons in range
            beacons = self.positioning.get_mean(self.bcn_buffer)
            for b in beacons:
                request = 'SRG ' + b + '\r'
                self.receiver_send_pub.publish(request)
                self.srg_wait = True
                while self.srg_wait:
                    rospy.sleep(0.1)

            self.rate.sleep()


if __name__ == '__main__':
    # start node
    rospy.init_node('positioning_plus', anonymous=True)
    # initialize IPSReceiver class
    ips = IPSplus()
    try:
        # publish receiver messages
        ips.publish()
    except rospy.ROSInterruptException:
        pass
