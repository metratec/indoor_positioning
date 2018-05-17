#!/usr/bin/env python
"""
Use this class to perform indoor positioning with the metraTec IPS system. The position of a receiver is evaluated by
the received messages of RF beacons placed in the environment. Requires a YAML file with the configuration of zones.

Usage:
Initialize the class by passing the directory of the config file. Then collect any number of beacon pings and use the
get_zone() function to return the current zone. Passing a list of pings instead of a single ping leads to averaging
over all the passed messages.
"""

from zone import Zone
from beacon import Beacon
import yaml


class Positioning:
    """Use the metraTec IPS system to perform positioning of a receiver depending on placement of RF beacons."""
    def __init__(self, config_dir):
        """
        Initialize class object by passing the directory to a YAML config file containing zone information.
        :param config_dir: String: directory of the YAML config file containing the definitions of zones
        """
        # store number of specified beacons
        self.n_beacons = 0
        # get data from config file and initialize zone objects
        self.zones = []
        self.parse_config(config_dir)

    def get_defined_beacons(self):
        """
        Return a list of EIDs of all beacons that are configured in the config file
        :return: [String]: EIDs of all beacons that are defined
        """
        eids = []
        # iterate over all zones
        for z in self.zones:
            # get every beacon in the zone
            for b in z.beacons:
                eids.append(b.eid)
        return eids

    def get_beacon(self, eid):
        """
        Return beacon object with the specified EID.
        :param eid: String: EID of the beacon object that should be fetched from the initialized zones
        :return: Beacon/None: beacon object with the specified EID
        """
        # iterate over all zones and search for beacon
        for z in self.zones:
            for b in z.beacons:
                if b.eid == eid:
                    return b
        # return None if no matching beacon was found
        return None

    def parse_config(self, yml_dir):
        """
        Read the YAML config files of zones, parse the data and create zone/beacon objects. The created zone objects
        are appended to the classes list of zones for later use.
        :param yml_dir: String: directory of the YAML config file containing the definitions of zones
        """
        # open YAML config file and store content as dictionary
        with open(yml_dir, 'r') as stream:
            yml = yaml.load(stream)

        # parse dictionary of YAML content and create list of zones
        for i in range(len(yml)):
            # get content of all zones in yml file and append to zones list
            z = 'zone' + str(i)  # dictionary keys for each zone (zone0, zone1, zone2, etc.)
            zone = yml[z]  # content of current zone
            # get polygon of current zone
            poly = []
            for j in range(0, len(zone['polygon']), 3):
                poly.append([zone['polygon'][j], zone['polygon'][j+1], zone['polygon'][j+2]])
            # get content of all beacons of current zone from beacon0, beacon1, beacon2, etc.
            beacons = []
            for k in range(len(zone)-4):  # -4 because of non-beacon entries (name, frame_id, polygon)
                b = 'beacon' + str(k)  # dictionary keys for each beacon inside the zone
                beacon = zone[b]  # content of current beacon
                beacons.append(Beacon(beacon['EID'], beacon['position'], zone['frame_id']))  # initialize Beacon object
                # increment number of beacons
                self.n_beacons += 1
            # append information about current zone to list and start with next iteration if more zones are defined
            self.zones.append(Zone(zone['name'], zone['frame_id'], zone['threshold'], poly, beacons))

    def get_zone(self, pings):
        """
        Take average RSSI value for each beacon ping contained in the passed list of beacon pings. Then return the
        zone which the beacon with the highest value belongs to.
        :param pings: List of Strings: beacon pings collected over time to average [BCN <EID> <RSSI>, ...]
        :return: Zone/None: zone object of beacons with highest signal strength
        """
        # get means of RSSI values for each individual beacon
        means = self.get_mean(pings)
        # sort mean dictionary by values and look for corresponding beacon/zone
        # if the top beacon is not found, continue with the next one until all beacons are exhausted
        for key, value in sorted(means.items(), key=lambda (k, v): v, reverse=True):
            # look for key (EID) in every zone and return when match is found
            for z in self.zones:  # iterate over zones
                # continue to next zone if mean RSSI value is lower than the configured threshold
                if value < z.threshold:
                    continue
                for b in z.beacons:  # iterate over beacons inside the zone
                    if b.eid == key:  # if the EID of the beacon is the same as the current key (EID) return the zone
                        return z
        # return None if no matching beacon has been found
        print('None of the received beacon pings match a beacon from the configuration file. Returning \'None\'')
        return None

    @staticmethod
    def get_mean(pings):
        """
        Compute the mean of RSSI values of a list of collected beacon pings.
        Sort the passed list of pings by their unique EID and allocate their measured RSSI values. Then compute and
        return the mean RSSI value of all received messages for each individual beacon.
        :param pings: List of Strings: beacon pings collected over time to average [BCN <EID> <RSSI>, ...]
        :return: Dict: key-value-pairs of EID and computed mean of RSSI values {<EID>: <AVERAGE_RSSI>, ...}
        """
        # create empty dictionary for <EID>: [<RSSI_VALUE>, ...] pairs
        ordered_values = dict()
        # loop through every entry in the list of received pings
        for p in pings:
            # remove end of line character
            if p[-1] == '\r':
                p = p[:-1]
            # split string on every SPACE
            split = p.split(' ')
            # check whether values are valid and skip current entry if they are not
            if len(split) != 3 or split[0] != 'BCN' or len(split[1]) != 16 or len(split[2]) != 4:
                print('Encountered invalid beacon ping: {}'.format(p))
                continue
            # append current RSSI value to respective entry in list if the current EID is already there
            if split[1] in ordered_values:
                ordered_values[split[1]].append(int(split[2]))
            # create new entry in for EID-RSSI-pairs if the current EID is not yet in the dictionary
            else:
                ordered_values[split[1]] = [int(split[2])]
        # compute average RSSI value for each beacon
        for o in ordered_values:
            ordered_values[o] = sum(ordered_values[o]) / len(ordered_values[o])
        # return mean RSSI values attached to their respective EIDs
        return ordered_values


if __name__ == '__main__':
    """Testing the class and its methods."""
    # initialize positioning class
    pos = Positioning('/home/metratec/catkin_ws/src/indoor_positioning/config/zones.yml')
    # create a list of pings that would usually be collected from the receiver and stored in a buffer
    dummy_pings = ['BCN 0123456789ABCDEF -060', 'BCN 0123456789ABCDEF -070', 'BCN 0123456789ABCDFF -070',
                   'BCN 0123456789ABCDFF -090', 'BCN 0123456789ABCFFF -070', 'BCN 0123456789ABCFFF -050',
                   'BCN 0123456789ABFFFF -070', 'BCN 0123456789ABFFFF -060', 'BCN 0123456789ABFFFF -090']
    # get the current zone of the receiver according to above list of beacon pings
    # Note: the above beacons have to be defined in the config file for this to work
    zone = pos.get_zone(dummy_pings)
    # break here to view objects and variables etc.
    breakpoint = 0
