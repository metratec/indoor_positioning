#!/usr/bin/env python
"""
DESCRIPTION
"""

from positioning import Positioning


class PositioningPlus:
    def __init__(self, config_dir):
        self.pos = Positioning(config_dir)

    def get_zone(self, pings):
        return self.pos.get_zone(pings)

    def get_mean(self, pings):
        return self.pos.get_mean(pings)

    def get_top_beacons(self, pings, n):
        means = self.pos.get_mean(pings)
        ret = []
        # sort mean dictionary by values and return n beacons with the highest signal strength that are configured in
        # the config-file and meet the threshold criteria
        for key, value in sorted(means.items(), key=lambda (k, v): v, reverse=True):
            # look for key (EID) in every zone and return when match is found
            for z in self.pos.zones:  # iterate over zones
                # continue to next zone if mean RSSI value is lower than the configured threshold
                if value < z.threshold:
                    continue
                for b in z.beacons:  # iterate over beacons inside the zone
                    if b.eid == key:  # if the EID of the beacon is the same as the current key (EID) return the zone
                        ret.append(b)
                        if len(ret) >= n:  # TODO continue with next key-value-pair when was found
                            return ret
        return ret


if __name__ == '__main__':
    # testing the class and its methods
    pos = PositioningPlus('/home/metratec/catkin_ws/src/ros_ips/config/zones.yml')
    dummy_pings = ['BCN 00124B00090593E6 -060', 'BCN 00124B00090593E6 -070', 'BCN 00124B00090593E6 -070',
                   'BCN 00124B00050CD41E -090', 'BCN 00124B00050CD41E -070', 'BCN 00124B00050CD41E -050',
                   'BCN 00124B00050CDC0A -070', 'BCN 00124B00050CDC0A -060', 'BCN 00124B00050CDC0A -090']
    zone = pos.get_zone(dummy_pings)
    top = pos.get_top_beacons(dummy_pings, 5)
    breakpoint = 0
