#!/usr/bin/env python
"""Use the USBRead class to establish a serial connection with a compatible device via USB port."""

from __future__ import print_function
import serial


# TODO use threading for serial connection via USB??
class USBRead:
    """Establish a serial connection with a connected USB device."""
    def __init__(self, port, baudrate=115200):
        """
        Initialize reader class for serial connection.
        :param port: String: USB port the device is connected to
        :param baudrate: Int: Baudrate to use for the communication with the USB device
        """
        # port of the device
        self.port = port
        # baudrate used for communication
        self.baudrate = baudrate
        # initialize serial connection object with the specified parameters
        self.ser = serial.Serial(self.port, self.baudrate)

    def open(self):
        """Open the serial connection."""
        # try do open the serial connection and throw error if not successful
        try:
            self.ser.open()
        except:
            print('Unable to open serial connection with: ' + self.port)

    def close(self):
        """Close the serial connection."""
        # try do close the serial connection and throw error if not successful
        try:
            self.ser.close()
        except:
            print ('Unable to close serial connection with: ' + self.port)

    def get_line(self):
        """
        Return a single line from the serial messages with b'\r' as the end-of-line-character.
        :return: Bytes: the most recent line that has been sent via the serial connection
        """
        # check whether port is open
        if not self.ser.isOpen():
            self.open()
        # define end-of-line-character
        eol = b'\r'
        # determine length of the eol-character
        leneol = len(eol)
        # initialize empty line as bytearray
        line = bytearray()
        # collect single bytes and concatenate line until eol-character is encountered
        while True:
            # read one byte
            c = self.ser.read(1)
            if c:
                # append to line
                line += c
                # break if the current character is the specified eol-character
                if line[-leneol:] == eol:
                    break
            else:
                break
        # return line
        return bytes(line)
