#!/usr/bin/env python
# TODO this is for USB and TCP now!!!
"""
Use this node to establish a connection with the metraTec IPS receiver USB stick. You have to pass the USB port the
stick is connected to as a command line argument or as a private parameter when you are using a launch file.
Running from command line:
    $ rosrun ros_ips receiver.py <PORT>
    For example:
    $ rosrun ros_ips receiver.py /dev/ttyUSB0

Running from a launch file:
    <node pkg="ros_ips" name="receiver" type="receiver.py">
        <param name="port" type="string" value="/dev/ttyUSB0"/>
    </node>
"""

from __future__ import print_function
import rospy
import sys
from std_msgs.msg import String
from ros_ips.msg import StringStamped
from ros_ips.communication.usb_serial import USBSerial
from ros_ips.communication.tcp_socket import TCPSocket


# TODO String -> StringStamped
class IPSReceiver:
    """
    Use this class to establish a connection with the metraTec IPS receiver or USB stick and publish the received
    messages to the ips/receiver/raw topic.
    """
    def __init__(self, type, required, optional):
        """
        Initialize IPS receiver class by passing the IP address the receiver or port the USB stick is connected to.
        :param type: String: type of the connection, either 'tcp' or 'usb'
        :param required: String: Either HOST_IP or USB_PORT of the receiver
        :param optional: Int: Either TCP_PORT or BAUDRATE to use for the connection
        """
        # type of connected to receiver. Either 'tcp' for standard receiver or 'usb' for USB stick
        self.type = type
        # required parameter for connection. IP address for standard receiver or USB port for USB stick
        self.req = required
        # optional parameter for connection. Port for standard receiver or Baudrate for USB stick
        self.opt = optional
        # initialize serial connection with USB stick or TCP connection with standard receiver
        if self.type == 'usb':
            self.con = USBSerial(self.req) if self.opt is None else USBSerial(self.req, baudrate=self.opt)
        elif self.type == 'tcp':
            self.con = TCPSocket(self.req) if self.opt is None else TCPSocket(self.req, port=self.opt)
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


def get_args(args):
    """
    Return required and optional parameters for this node. Parameters are taken from ROS parameter server if available,
    otherwise from command line arguments. Required arguments are either HOST_IP or USB_PORT. Depending on the required
    parameter the optional parameter is either a TCP_PORT or the BAUDRATE, respectively.
    :param args: CommandLineArguments: arguments passed after $rosrun receiver
    :return: String/None: required parameter (HOST_IP for TCP/IP connection, USB_PORT for USB connection)
             Int/None: optional parameter (TCP_PORT for TCP/IP connection, BAUDRATE for USB connection)
    """
    type, req, opt = None, None, None
    if rospy.has_param('~host'):
        type = 'tcp'
        req = str(rospy.get_param('~host'))
        if rospy.has_param('~port'):
            opt = int(rospy.get_param('~port'))
    elif rospy.has_param('~port'):
        type = 'usb'
        req = str(rospy.get_param('~port'))
        if rospy.has_param('~baudrate'):
            opt = int(rospy.get_param('~baudrate'))
    elif len(args) > 2:
        type = str(args[1])
        req = str(args[2])
        if len(args) > 3:
            opt = int(args[3])

    return type, req, opt


if __name__ == '__main__':
    # start node
    rospy.init_node('receiver', anonymous=True, disable_signals=True)
    # get parameters from command line arguments or parameter server
    type, req, opt = get_args(sys.argv)
    # initialize IPSReceiver class
    ips = IPSReceiver(type, req, opt)
    try:
        # publish receiver messages
        ips.publish()
    except rospy.ROSInterruptException:
        pass
