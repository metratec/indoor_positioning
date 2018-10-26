#!/usr/bin/env python
"""
Use this node to establish a connection with the metraTec IPS receiver or USB stick. You have to pass the USB port the
stick is connected to or the IP address of the regular receiver as a command line argument or as a private parameter
when you are using a launch file.

Running from command line:
    $ rosrun indoor_positioning receiver.py <TYPE> <REQUIRED> <OPTIONAL>
    For example:
    $ rosrun indoor_positioning receiver.py usb /dev/ttyUSB0
    $ rosrun indoor_positioning receiver.py tcp 192.168.2.223

Subscribed topics:
    - ips/receiver/send (std_msgs/String):
        Message to be sent over the TCP or serial connection to the IPS receiver or USB stick

Published topics:
    - ips/receiver/raw (indoor_positioning/StringStamped):
        Raw messages received from the receiver or USB stick

Parameters:
    - ~type (string, default='tcp'):
        'tcp' for TCP receiver or 'usb' for metraTec USB stick
    - ~host (string, default='192.168.2.223):
        IP address of the connected receiver
    - ~port (with type='tcp': int, default=10001; with type='usb': string, default='/dev/ttyUSB0):
        Port for the TCP connection if '~type' parameter is set to 'tcp', otherwise specifies the USB port of the stick
    - ~baudrate (int, default=115200):
        Baudrate to use for serial communication with USB receiver
"""

from __future__ import print_function
import rospy
from std_msgs.msg import String
from indoor_positioning.msg import StringStamped
from indoor_positioning.communication.usb_serial import USBSerial
from indoor_positioning.communication.tcp_socket import TCPSocket


class IPSReceiver:
    """
    Use this class to establish a connection with the metraTec IPS receiver or USB stick and publish the received
    messages to the ips/receiver/raw topic. The node also listens to the ips/receiver/send topic and invokes a
    callback that forwards these messages to the connected receiver when it is updated.
    """
    def __init__(self):
        """Initialize IPS receiver class by passing the IP address the receiver or port the USB stick is connected to."""
        # type of connected to receiver. Either 'tcp' for standard receiver or 'usb' for USB stick
        self.type = rospy.get_param('~type') if rospy.has_param('~type') else 'tcp'
        # required parameter for connection. IP address for standard receiver or USB port for USB stick
        self.host = rospy.get_param('~host') if rospy.has_param('~host') else '192.168.2.223'
        self.port = rospy.get_param('~port') if rospy.has_param('~port') else 10001
        # initialize serial connection with USB stick or TCP connection with standard receiver
        if self.type == 'usb':
            self.baudrate = rospy.get_param('~baudrate') if rospy.has_param('~baudrate') else 115200
            self.con = USBSerial(self.port, baudrate=self.baudrate)
        elif self.type == 'tcp':
            self.con = TCPSocket(self.host, port=int(self.port))
        else:
            print('Connection type "{}" is invalid. Shutting down node!'.format(self.type))
            rospy.signal_shutdown('Connection type "{}" is invalid. Shutting down node!'.format(self.type))

        # initialize subscriber that listens to ips/receiver/send to send messages to the receiver on callback
        self.rec_send_sub = rospy.Subscriber('ips/receiver/send', String, self.callback)
        # initialize publisher object
        self.rec_pub = rospy.Publisher('ips/receiver/raw', StringStamped, queue_size=10)
        # specify publishing rate
        self.rate = rospy.Rate(30)

    def callback(self, msg):
        """
        Send message to connected device when /ips/receiver/send topic is updated.
        :param msg: String: message to be sent to the connected device
        """
        self.con.send(msg.data)
        print(msg.data)
        # rospy.sleep(0.5)

    def publish(self):
        """Publish raw receiver messages to the ips/receiver/raw topic."""
        while not rospy.is_shutdown():
            # get a single receiver message
            line = self.con.get_line(b'\r')
            # generate message
            msg = StringStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = str(line)
            # publish line as String message with timestamp
            self.rec_pub.publish(msg)
            # wait according to publishing rate
            self.rate.sleep()

    def shutdown(self):
        """Close USB connection on shutdown."""
        self.con.close()


if __name__ == '__main__':
    # start node
    rospy.init_node('receiver', anonymous=True, disable_signals=True)
    # initialize IPSReceiver class
    ips = IPSReceiver()
    try:
        # publish receiver messages
        ips.publish()
    except rospy.ROSInterruptException:
        pass
