#!/usr/bin/env python
"""
Use this node to perform indoor positioning using the metraTec IPS+ tracking system. Prerequisites for using this node
is a running receiver-node that handles communication with the receiver and thus with the beacons in the vicinity.
Also, make sure that tf is broadcasting transformations between all the different coordinate frames used in the config
file and the coordinate frame of the receiver (which is specified via rosparam, see below).

Subscribed topics:
    - ips/receiver/raw (indoor_positioning/StringStamped):
        Raw messages received by the UWB receiver

Published topics:
    - ips/receiver/send (std_msgs/String):
        Message to be sent to the receiver, e.g. 'SRG <EID> \r' ranging request
    - ips/receiver/position (geometry_msgs/PointStamped):
        Estimated position of the receiver after UWB ranging and trilateration

Parameters:
    - ~config_file (string, default='PKG_DIR/config/zones.yml'):
        Path to the configuration file of zones and beacons relative to the package directory
    - ~frame_id (string, default='map'):
        Coordinate frame the receiver position should be estimated in
    - ~rate (double, default=0.1):
        The publishing rate in messages per second
    - ~bcn_len (int, default=2*number_of_beacons):
        Buffer length for BCN messages
    - ~srg_len (int, default=number_of_beacons):
        Buffer length for SRG messages
    - ~min_beacons (int, default=4):
        Minimum number of beacons to be used for UWB ranging. Should be 3 (two possible points) or 4
    - ~max_z (double, default=None):
        Maximum z-coordinate the receiver should have after ranging. Used as bounds for trilateration.
"""

import os
import rospy
import rospkg
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from indoor_positioning.msg import StringStamped
from indoor_positioning.positioning_plus import PositioningPlus


class IPSplus:
    """Configure ROS node for metraTec IPS+ indoor positioning system with UWB ranging functionality."""
    # parameters specifying wait for SRG responses and timeout to avoid infinite wait for SRG response
    srg_sleep = 0.1  # interval in which to check for responses in seconds
    srg_timeout = 20  # number of times to check for responses. After that, continue with next beacon

    def __init__(self):
        """
        Initialize instance variables with values from ROS parameter server (or default values) and zone/beacon
        configuration from YAML file.
        """
        # get directory of config file
        config_dir = rospy.get_param('~config_file') if rospy.has_param('~config_file') else 'config/zones.yml'
        abs_dir = os.path.join(rospkg.RosPack().get_path('indoor_positioning'), config_dir)
        # get minimum number of beacons to use for ranging
        min_beacons = rospy.get_param('~min_beacons') if rospy.has_param('~min_beacons') else 4
        # get maximum z position the receiver can have
        max_z = rospy.get_param('~max_z') if rospy.has_param('~max_z') else None
        # initialize positioning class
        self.positioning = PositioningPlus(abs_dir, min_beacons=min_beacons, max_z=max_z)
        # get number of beacons specified in zones.yml file for default buffer values
        n_beacons = self.positioning.n_beacons

        # publisher for estimated position of UWB trilateration
        self.position_pub = rospy.Publisher('ips/receiver/position', PointStamped, queue_size=1)
        # publisher for serial messages to be sent to the receiver
        self.receiver_send_pub = rospy.Publisher('ips/receiver/send', String, queue_size=n_beacons)
        # subscribe to raw messages of receiver
        self.receiver_sub = rospy.Subscriber('ips/receiver/raw', StringStamped, self.callback)

        # number of messages to keep
        self.bcn_buffer_length = rospy.get_param('~bcn_len') if rospy.has_param('~bcn_len') else 2*n_beacons
        # list of incoming messages
        self.bcn_buffer = []
        # timestamp from last received message
        self.bcn_last_time = None

        # number of messages to keep
        self.srg_buffer_length = rospy.get_param('~srg_len') if rospy.has_param('srg_len') else n_beacons
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

        # set publishing rate
        self.rate = rospy.Rate(rospy.get_param('~rate')) if rospy.has_param('~rate') else rospy.Rate(0.1)

    def callback(self, msg):
        """
        Append incoming messages to list of previous message. Differentiate between BCN messages (regular beacon pings)
        and SRG messages (responses of UWB ranging responses)
        :param msg: String, message of subscribed topic
        """
        # sort message and add to buffer depending on prefix of the message
        if msg.data.split(' ')[0] == 'BCN':
            buff_len = self.bcn_buffer_length
            buff = self.bcn_buffer
            self.bcn_last_time = msg.header.stamp
        elif msg.data.split(' ')[0] == 'SRG':
            buff_len = self.srg_buffer_length
            buff = self.srg_buffer
            self.srg_last_time = msg.header.stamp
            self.srg_wait = False
        else:
            return
        # append message to buffer
        buff.append(msg.data)
        # delete oldest message if buffer is full
        if len(buff) > buff_len:
            del(buff[0])

    def publish(self):
        """Publish the estimated position of the receiver"""
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
        """
        Estimate the position of the receiver using UWB ranging responses.
        :return: [Float, Float, Float]: estimated position of the UWB receiver [x, y, z]
        """
        while not rospy.is_shutdown():
            # get all beacons in range
            beacons = self.positioning.in_range(self.bcn_buffer)
            print('Using the following beacons for ranging: {}'.format(beacons))
            # send ranging request to all beacons
            iters = 0
            for b in beacons:
                request = 'SRG ' + b + '\r'
                self.srg_wait = True
                self.receiver_send_pub.publish(request)
                # wait until SRG response arrives
                while self.srg_wait:
                    # stop waiting after a specific amount of iterations to avoid infinite loop
                    if iters >= self.srg_timeout:
                        print('Abort ranging with {} due to timeout.'.format(b))
                        break
                    # wait for SRG response
                    rospy.sleep(self.srg_sleep)
                    # increment number of iterations
                    iters += 1
                # reset number of iterations for next beacon
                iters = 0
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
                        (trans, rot) = self.tf.lookupTransform('/' + self.frame_id, '/' + r[0].frame_id, rospy.Time(0))
                        # transform point
                        tp = r[0].position[:]
                        tp.append(0)
                        tp = tf.transformations.quaternion_multiply(
                            tf.transformations.quaternion_multiply(rot, tp),
                            tf.transformations.quaternion_conjugate(rot)
                        )
                        tp = [tp[0]+trans[0], tp[1]+trans[1], tp[2]+trans[2]]
                        print(tp)
                        transformed_ranges.append((tp, r[1]))
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        print('Cannot transform {} into {}. Check your tf tree!'.format(r[0].frame_id, self.frame_id))
                        continue
                else:
                    transformed_ranges.append((r[0].position, r[1]))
            # estimate position
            position = self.positioning.trilaterate(transformed_ranges)
            # clear SRG message buffer
            self.srg_buffer = []
            # return estimated position
            return position


if __name__ == '__main__':
    # start node
    rospy.init_node('positioning_plus', anonymous=False)
    # initialize IPSReceiver class
    ips = IPSplus()
    try:
        # publish receiver messages
        ips.publish()
    except rospy.ROSInterruptException:
        pass
