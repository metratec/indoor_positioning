#!/usr/bin/env python
"""
DESCRIPTION
"""

import rospy
import os
from geometry_msgs.msg import PolygonStamped, Point32
from ros_ips.msg import StringStamped
from ros_ips.positioning import Positioning


class IPSMap:
    """
    DESCRIPTION
    """
    def __init__(self):
       pass

    def publish(self):
        """Publish static tfs for beacon positions and zone polygons"""
        pass


if __name__ == '__main__':
    # start node
    rospy.init_node('ips_map', anonymous=True)
    # initialize IPSReceiver class
    ips = IPSMap()
    try:
        # publish receiver messages
        ips.publish()
    except rospy.ROSInterruptException:
        pass
