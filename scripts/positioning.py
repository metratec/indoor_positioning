#!/usr/bin/env python
"""
DESCRIPTION
"""

import rospy
import os
from geometry_msgs.msg import PolygonStamped, Point32
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
        self.buffer_length = 4  # TODO make configurable
        # list of incoming messages
        self.msg_buffer = []
        # timestamp from last received message
        self.last_time = None

        # initialize positioning class
        config_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config/zones.yml')
        self.positioning = Positioning(config_dir)

        # publishers
        # current zone name
        self.zone_name_pub = rospy.Publisher('ips/receiver/current_zone/name', StringStamped, queue_size=1)
        # polygon of current zone
        self.zone_polygon_pub = rospy.Publisher('ips/receiver/current_zone/polygon', PolygonStamped, queue_size=1)
        # zone leave event
        self.zone_leave_pub = rospy.Publisher('ips/receiver/zone_leave', StringStamped, queue_size=1)
        # zone enter event
        self.zone_enter_pub = rospy.Publisher('ips/receiver/zone_enter', StringStamped, queue_size=1)
        # set publishing rate
        self.rate = rospy.Rate(1)

    def callback(self, msg):
        """
        Append incoming messages to list of previous messages.
        :param msg: String, message of subscribed topic
        """
        # append message to buffer
        self.msg_buffer.append(msg.data)
        # save time of last raw signal
        self.last_time = msg.header.stamp
        # delete oldest message if buffer is full
        if len(self.msg_buffer) > self.buffer_length:
            del(self.msg_buffer[0])

    def publish(self):
        """Publish zone information"""
        # last zone that the receiver was in
        last_zone = None
        while not rospy.is_shutdown():
            # get the current zone
            zone = self.positioning.get_zone(self.msg_buffer) if self.msg_buffer else None
            # check if zone change occurred
            if zone != last_zone:
                # publish zone change event  # TODO remove prints
                event = StringStamped()
                event.header.stamp = self.last_time
                if zone is None:
                    event.data = last_zone.name
                    self.zone_leave_pub.publish(event)
                    print('zone leave')
                elif last_zone is None:
                    event.data = zone.name
                    self.zone_enter_pub.publish(event)
                    print('zone enter')
                else:
                    event.data = last_zone.name
                    self.zone_leave_pub.publish(event)
                    event.data = zone.name
                    self.zone_enter_pub.publish(event)
                    print('zone leave and enter')
            if zone is not None:  # TODO publish 'None' when no zone was found?
                # publish zone name
                name = StringStamped()
                name.header.stamp = self.last_time
                name.header.frame_id = zone.frame_id
                name.data = zone.name
                self.zone_name_pub.publish(name)
                # publish zone polygon
                polygon = PolygonStamped()
                polygon.header.stamp = self.last_time
                polygon.header.frame_id = 'map'  # TODO get from zone class as specified in config file
                points = []
                for p in zone.polygon:
                    points.append(Point32(p[0], p[1], p[2]))
                polygon.polygon.points = points
                self.zone_polygon_pub.publish(polygon)
            # set current zone to last zone
            last_zone = zone
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
