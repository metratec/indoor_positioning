#!/usr/bin/env python
"""
Use this node to perform indoor zone location using the metraTec IPS tracking system. Prerequisites for using this node
is a running receiver-node that handles communication with the receiver and thus with the beacons in the vicinity.
Also, make sure that you have defined your zones correctly in the YAML config file.

Subscribed topics:
    - ips/receiver/raw (ros_ips/StringStamped):
        Raw messages received by the UWB receiver

Published topics:
    - ips/receiver/current_zone/name (ros_ips/StringStamped):
        Name of the zone the receiver is currently in
    - ips/receiver/current_zone/polygon (geometry_msgs/PolygonStamped):
        Polygon comprising the current zone
    - ips/receiver/zone_leave (ros_ips/StringStamped):
        Name of the zone that the receiver has left. Is published at the moment a zone-leave occurs
    - ips/receiver/zone_enter (ros_ips/StringStamped):
        Name of the zone that the receiver has entered. Is published at the moment a zone-enter occurs

Parameters:
    - ~config_file (string, default='PKG_DIR/config/zones.yml'):
        Path to the configuration file of zones and beacons relative to the package directory
    - ~rate (double, default=1):
        The publishing rate in messages per second
    - ~bcn_len (int, default=2*number_of_beacons):
        Buffer length for BCN messages
"""

import rospy
import os
import rospkg
from geometry_msgs.msg import PolygonStamped, Point32
from ros_ips.msg import StringStamped
from ros_ips.positioning import Positioning


class IPS:
    """Configure ROS node for metraTec IPS indoor positioning system for zone location."""
    def __init__(self):
        # subscribe to raw messages from USB stick
        self.receiver_sub = rospy.Subscriber('ips/receiver/raw', StringStamped, self.callback)

        # get directory of config file
        config_dir = rospy.get_param('~config_file') if rospy.has_param('~config_file') else 'config/zones.yml'
        abs_dir = os.path.join(rospkg.RosPack().get_path('ros_ips'), config_dir)
        # initialize positioning class
        self.positioning = Positioning(abs_dir)
        # get number of beacons specified in zones.yml file for default buffer values
        n_beacons = self.positioning.n_beacons

        # number of messages to keep
        self.buffer_length = rospy.get_param('~bcn_len') if rospy.has_param('~bcn_len') else 2*n_beacons
        # list of incoming messages
        self.msg_buffer = []
        # timestamp from last received message
        self.last_time = None

        # publishers
        # current zone name
        self.zone_name_pub = rospy.Publisher('ips/receiver/current_zone/name', StringStamped, queue_size=1)
        # polygon of current zone
        self.zone_polygon_pub = rospy.Publisher('ips/receiver/current_zone/polygon', PolygonStamped, queue_size=1)
        # zone leave event
        self.zone_leave_pub = rospy.Publisher('ips/receiver/zone_leave', StringStamped, queue_size=10)
        # zone enter event
        self.zone_enter_pub = rospy.Publisher('ips/receiver/zone_enter', StringStamped, queue_size=10)
        # set publishing rate
        self.rate = rospy.Rate(rospy.get_param('~rate')) if rospy.has_param('~rate') else rospy.Rate(1)

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
                # publish zone change event
                event = StringStamped()
                event.header.stamp = self.last_time
                # only zone leave
                if zone is None:
                    event.data = last_zone.name
                    self.zone_leave_pub.publish(event)
                # only zone enter
                elif last_zone is None:
                    event.data = zone.name
                    self.zone_enter_pub.publish(event)
                # leave on zone and enter another
                else:
                    event.data = last_zone.name
                    self.zone_leave_pub.publish(event)
                    event.data = zone.name
                    self.zone_enter_pub.publish(event)
            if zone is not None:
                # publish zone name
                name = StringStamped()
                name.header.stamp = self.last_time
                name.header.frame_id = zone.frame_id
                name.data = zone.name
                self.zone_name_pub.publish(name)
                # publish zone polygon
                polygon = PolygonStamped()
                polygon.header.stamp = self.last_time
                polygon.header.frame_id = zone.frame_id
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
    rospy.init_node('positioning', anonymous=False)
    # initialize IPSReceiver class
    ips = IPS()
    try:
        # publish receiver messages
        ips.publish()
    except rospy.ROSInterruptException:
        pass
