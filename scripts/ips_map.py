#!/usr/bin/env python
"""
This node publishes tf transforms between the frame_id set in the config file and the beacon positions in that zone.
You can use these transforms for visualization in rviz to make sure you have set up your environment correctly.

Published tf's:
    - beacon.frame_id -> beacon.eid
    Transforms for all zones and beacons defined in the config file. Transforms are broadcasted from the zone
    frame_id to the position of the beacon in that frame

Parameters:
    - ~config_file (string, default='config/zones.yml'):
    Path to the configuration file of zones and beacons relative to the package directory
    - ~rate (double, default=0.1):
    The publishing rate of transforms in transforms per second
"""

import os
import rospy
import tf2_ros
import rospkg
from geometry_msgs.msg import TransformStamped
from indoor_positioning.positioning import Positioning


class IPSMap:
    """
    Get all beacons from the configuration file and store them in a list. Then publish tf transforms for every beacon
    from the zone's frame_id to the position of the beacon.
    """
    def __init__(self):
        # get directory of config file
        config_dir = rospy.get_param('~config_file') if rospy.has_param('~config_file') else 'config/zones.yml'
        abs_dir = os.path.join(rospkg.RosPack().get_path('indoor_positioning'), config_dir)
        # initialize positioning class
        positioning = Positioning(abs_dir)

        # get all beacon objects
        self.beacons = []
        for z in positioning.zones:
            for b in z.beacons:
                self.beacons.append(b)

        # initialize transform broadcaster
        self.br = tf2_ros.StaticTransformBroadcaster()

        # publishing rate
        self.rate = rospy.Rate(rospy.get_param('~rate')) if rospy.has_param('~rate') else rospy.Rate(0.1)

    def publish(self):
        """Publish static tfs for beacon positions and zone polygons"""
        tf = TransformStamped()
        while not rospy.is_shutdown():
            # publish transforms for all positions
            tf.header.stamp = rospy.Time.now()
            for b in self.beacons:
                tf.header.frame_id = b.frame_id
                tf.child_frame_id = b.eid

                tf.transform.translation.x = b.position[0]
                tf.transform.translation.y = b.position[1]
                tf.transform.translation.z = b.position[2]

                tf.transform.rotation.x = 0
                tf.transform.rotation.y = 0
                tf.transform.rotation.z = 0
                tf.transform.rotation.w = 1

                self.br.sendTransform(tf)
                rospy.sleep(0.5)
            # wait for next iteration
            self.rate.sleep()


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
