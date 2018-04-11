#!/usr/bin/env python
"""
DESCRIPTION
"""

from zone import Zone
from beacon import Beacon
import yaml


class Positioning:
    def __init__(self, config_dir):
        # get data from config file and initialize zone objects
        self.zones = []
        self.parse_config(config_dir)

    def parse_config(self, yml_dir):
        # open YAML config file and store content as dictionary
        with open(yml_dir, 'r') as stream:
            yml = yaml.load(stream)

        # parse dictionary of YAML content and create list of zones
        for i in range(len(yml)):
            # get content of all zones in yml file and append to zones list
            z = 'zone' + str(i)  # dictionary keys for each zone (zone0, zone1, zone2, etc.)
            zone = yml[z]  # content of current zone
            # get content of all beacons of current zone from beacon0, beacon1, beacon2, etc.
            beacons = []
            for k in range(len(zone)-3):  # -3 because of non-beacon entries (name, frame_id, polygon)
                b = 'beacon' + str(k)  # dictionary keys for each beacon inside the zone
                beacon = zone[b]  # content of current beacon
                beacons.append(Beacon(beacon['EID'], beacon['position']))  # initialize Beacon object
            # append information about current zone to list and start with next iteration if more zones are defined
            self.zones.append(Zone(zone['name'], zone['frame_id'], zone['polygon'], beacons))

    def get_zone(self, pings):
        """DESCRIPTION Get zone from a list of pings"""
        # TODO check if messages are correct -> BCN -> SPACE -> 16 characters -> SPACE -> 4 characters
        # TODO average over RSSI values
        # TODO pick beacon with highest average
        # TODO look for beacon in zones and return respective zone info


if __name__ == '__main__':
    # testing the class and its methods
    p = Positioning('/home/metratec/catkin_ws/src/ros_ips/config/zones.yml')
    dummy_pings = ['BCN 0123456789ABCDEF -060', 'BCN 0123456789ABCDEF -070', 'BCN 0123456789ABCDFF -070',
                   'BCN 0123456789ABCDFF -090', 'BCN 0123456789ABCFFF -070', 'BCN 0123456789ABCFFF -050',
                   'BCN 0123456789ABFFFF -070', 'BCN 0123456789ABFFFF -060', 'BCN 0123456789ABFFFF -090']
    # TODO get zone
    a = 0