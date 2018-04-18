#!/usr/bin/env python
"""Use the TCPSocket class to establish a serial connection with a compatible device via USB port."""

from __future__ import print_function
import socket


# TODO use threading??
class TCPSocket:
    """Establish a TCP/IP connection with a connected device."""
    def __init__(self, host, port=10001):
        """
        Initialize reader class for serial connection.
        :param host: String: IP address of the device you want to communicate with
        :param port: Int (default=10001): port to use for the connection
        """
        # port of the device
        self.host = host
        # baudrate used for communication
        self.port = port
        # initialize serial connection object with the specified parameters
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))

    def open(self):
        """Open a socket to establish TCP/IP connection."""
        # try do open the serial connection and throw error if not successful
        try:
            self.sock.connect((self.host, self.port))
        except:
            print('Unable to open socket connection with: ' + self.host + ', ' + self.port)

    def close(self):
        """Close the socket."""
        # try do close the serial connection and throw error if not successful
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
            self.sock.close()
        except:
            print('Unable to close socket connection with: ' + self.host + ', ' + self.port)

    def send(self, msg):
        """
        Send message to the socket.
        :param msg: String: message to be sent to the socket
        """
        try:
            self.sock.send(msg)
        except:
            print('Unable to send message to socket.')

    def get_line(self, eol_character):
        """
        Return a single line from the TCP stream with a custom end-of-line character.
        :param eol_character: Bytes: character or sequence to use as end-of-line character.
        :return: Bytes: the most recent line that has been sent via the socket connection
        """
        # define end-of-line-character
        eol = eol_character
        # determine length of the eol-character
        leneol = len(eol)
        # initialize empty line as bytearray
        line = bytearray()
        # collect single bytes and concatenate line until eol-character is encountered
        while True:
            # read one byte
            c = self.sock.recv(1)
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