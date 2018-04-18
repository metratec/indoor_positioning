#!/usr/bin/env python
"""
DESCRIPTION
"""

import rospy
from std_msgs.msg import String
from ros_ips.msg import StringStamped
from ros_ips.positioning_plus import PositioningPlus


class IPSplus:
    def __init__(self):
        # publisher for serial messages to be sent to the receiver
        self.receiver_send_pub = rospy.Publisher('ips/receiver/send', String, queue_size=1)
        # subscribe to raw messages of receiver
        self.receiver_sub = rospy.Subscriber('ips/receiver/raw', StringStamped, self.callback)

        # set publishing rate
        self.rate = rospy.Rate(0.5)

    def callback(self, msg):
        """
        DESCRIPTION
        """
        print(msg.data)

    def publish(self):
        while not rospy.is_shutdown():
            self.receiver_send_pub.publish('SRG 00124B00090593E6\r')
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
