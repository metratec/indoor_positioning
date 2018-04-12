#!/usr/bin/env python
"""
DESCRIPTION
"""

import rospy
import os
from std_msgs.msg import String
from ros_ips.msg import StringStamped
from ros_ips.positioning import Positioning


class IPS:
    """
    DESCRIPTION
    """
    def __init__(self):
        # subscribe to raw messages from USB stick
        self.beacon_sub = rospy.Subscriber('ips/receiver/raw', StringStamped, self.callback)

        # number of messages to keep
        self.buffer_length = 6
        # list of incoming messages
        self.msg_buffer = []

        # initialize positioning class
        config_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config/zones.yml')
        self.positioning = Positioning(config_dir)

        # publishers
        # current zone name
        self.zone_name_pub = rospy.Publisher('ips/receiver/current_zone/name', String, queue_size=1)
        # set publishing rate
        self.rate = rospy.Rate(1)

    def callback(self, msg):
        """
        Append incoming messages to list of previous messages.
        :param msg: String, message of subscribed topic
        """
        # append message to buffer
        self.msg_buffer.append(msg.data)
        # delete oldest message if buffer is full
        if len(self.msg_buffer) > self.buffer_length:
            del(self.msg_buffer[0])

    def publish(self):
        """Publish zone information"""
        while not rospy.is_shutdown():
            zone = self.positioning.get_zone(self.msg_buffer) if self.msg_buffer else None
            if zone is not None:
                self.zone_name_pub.publish(zone.name)
            # wait to start next iteration
            self.rate.sleep()


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
