#!/usr/bin/env python
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

import rospy
import sys
from ros_ips.msg import StringStamped
from ros_ips.serial_com import USBRead


# TODO String -> StringStamped
class IPSReceiver:
    """
    Use this class to establish a connection with the metraTec IPS receiver USB stick and publish the received
    messages to the ips/receiver/raw topic.
    """
    def __init__(self, port):
        """
        Initialize IPS receiver class by passing the port the USB stick is connected to.
        :param port: String: USB port the receiver is connected to
        """
        self.port = port
        # initialize serial connection with USB stick
        self.usb = USBRead(port)
        # initialize publisher object
        self.rec_pub = rospy.Publisher('ips/receiver/raw', StringStamped, queue_size=1)
        # specify publishing rate
        self.rate = rospy.Rate(30)

    def publish(self):
        """Publish raw receiver messages to the ips/receiver/raw topic."""
        while not rospy.is_shutdown():
            # get a single receiver message
            line = self.usb.get_line()
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
        self.usb.close()


def get_args(args):
    """
    Return the USB port connected to the IPS receiver. Parameter server has priority over command line arguments.
    :param args: CommandLineArguments: arguments passed after $rosrun receiver
    :return: String/None: USB port the stick is connected to
    """
    # check if ROS parameter server has required parameters
    if rospy.has_param('~port'):
        port = str(rospy.get_param('~port'))
    # take command line arguments if available
    elif len(args) > 1:
        port = str(args[1])
    # return None if no parameters are available
    else:
        port = None

    return port


if __name__ == '__main__':
    # start node
    rospy.init_node('receiver', anonymous=True)
    # get parameters from command line arguments or parameter server
    port = get_args(sys.argv)
    # initialize IPSReceiver class
    ips = IPSReceiver(port)
    try:
        # publish receiver messages
        ips.publish()
    except rospy.ROSInterruptException:
        pass
