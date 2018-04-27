#!/usr/bin/env python
"""
DESCRIPTION
"""

import os
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from ros_ips.positioning import Positioning


class IPSMap:
    """
    DESCRIPTION
    """
    def __init__(self):
        # get directory of config file
        config_dir = rospy.get_param('~config_file') if rospy.has_param('~config_file') else 'config/zones.yml'
        abs_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), config_dir)
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
                rospy.sleep(0.1)
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
