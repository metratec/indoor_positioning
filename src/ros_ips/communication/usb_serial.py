#!/usr/bin/env python
"""Use the USBSerial class to establish a serial connection with a compatible device via USB port."""

from __future__ import print_function
import serial


# TODO use threading for serial connection via USB??
class USBSerial:
    """Establish a serial connection with a connected USB device."""
    def __init__(self, port, baudrate=115200):
        """
        Initialize class for serial connection.
        :param port: String: USB port the device is connected to
        :param baudrate (default=115200): Int: Baudrate to use for the communication with the USB device
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
            print('Unable to close serial connection with: ' + self.port)

    def send(self, msg):
        """
        Send serial message over the established connection.
        :param msg: String: message to be sent over serial connection
        """
        try:
            self.ser.write(msg)
        except:
            print('Unable to send message over serial connection.')

    def get_line(self, eol_character):
        """
        Return a single line from the serial messages with a custom end-of-line character.
        :param eol_character: Bytes: character or sequence to use as end-of-line character.
        :return: Bytes: the most recent line that has been sent via the serial connection
        """
        # check whether port is open
        if not self.ser.isOpen():
            self.open()
        # define end-of-line-character
        eol = eol_character
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
