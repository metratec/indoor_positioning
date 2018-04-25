#!/usr/bin/env python
"""
DESCRIPTION
queue_size should probably be at least equal to number of beacons
buffer_length should probably be at least 2x number of beacons
"""

import os
import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from ros_ips.msg import StringStamped
from ros_ips.positioning_plus import PositioningPlus


class IPSplus:
    def __init__(self):
        # publisher for estimated position of UWB trilateration
        self.position_pub = rospy.Publisher('ips/receiver/position', PointStamped, queue_size=1)
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

        # initialize tf transform listener
        self.tf = tf.TransformListener()
        # get map frame to do positioning in from parameter server. Defaults to "map" if not specified
        self.frame_id = rospy.get_param('~frame_id') if rospy.has_param('~frame_id') else 'map'

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
        buff.append(msg.data)
        # delete oldest message if buffer is full
        if len(buff) > buff_len:
            del(buff[0])
        # TODO docs

    def publish(self):
        while not rospy.is_shutdown():
            # estimate current position
            position = self.estimate_position()
            # publish estimated position
            if position is not None and len(position) == 3:
                point = PointStamped()
                point.header.stamp = rospy.Time.now()
                point.header.frame_id = self.frame_id
                point.point.x, point.point.y, point.point.z = position[0], position[1], position[2]
                self.position_pub.publish(point)
            self.rate.sleep()

    def estimate_position(self):
        while not rospy.is_shutdown():
            # get all beacons in range
            beacons = self.positioning.get_mean(self.bcn_buffer)
            # send ranging request to all beacons
            for b in beacons:
                request = 'SRG ' + b + '\r'
                self.receiver_send_pub.publish(request)
                self.srg_wait = True
                # wait until SRG response arrives
                while self.srg_wait:
                    rospy.sleep(0.1)
            # get ranges for all SRG responses
            ranges = self.positioning.parse_srg(self.srg_buffer)
            # transform beacon positions into frame specified by self.frame_id (~frame_id)
            transformed_ranges = []
            for i, r in enumerate(ranges):
                # skip points that are already in the correct frame
                if r[0].frame_id != self.frame_id:
                    # transform points into correct frame if tf knows the respective transform
                    try:
                        # look for transform from beacon frame to receiver frame
                        (trans, rot) = self.tf.lookupTransform('/' + r[0].frame_id, '/' + self.frame_id, rospy.Time(0))
                        # transform point
                        tp = r[0].position[:]
                        tp.append(0)
                        tp = tf.transformations.quaternion_multiply(
                            tf.transformations.quaternion_multiply(rot, tp),
                            tf.transformations.quaternion_conjugate(rot)
                        )
                        tp = [tp[0]+trans[0], tp[1]+trans[1], tp[2]+trans[2]]
                        transformed_ranges.append((tp, r[1]))
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        print('Cannot transform {} into {}. Check your tf tree!'.format(r[0].frame_id, self.frame_id))
                        continue
                else:
                    transformed_ranges.append((r[0].position, r[1]))
            return self.positioning.trilaterate(transformed_ranges)


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
