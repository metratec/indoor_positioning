#!/usr/bin/env python
"""
DESCRIPTION
"""

import math
from scipy.optimize import minimize
from positioning import Positioning


class PositioningPlus(Positioning):
    def __init__(self, config_dir):
        Positioning.__init__(self, config_dir)

    def get_top_beacons(self, pings, n):
        means = self.get_mean(pings)
        ret = []
        # sort mean dictionary by values and return n beacons with the highest signal strength that are configured in
        # the config-file and meet the threshold criteria
        for key, value in sorted(means.items(), key=lambda (k, v): v, reverse=True):
            # look for key (EID) in every zone and return when match is found
            for z in self.zones:  # iterate over zones
                # continue to next zone if mean RSSI value is lower than the configured threshold
                if value < z.threshold:
                    continue
                for b in z.beacons:  # iterate over beacons inside the zone
                    if b.eid == key:  # if the EID of the beacon is the same as the current key (EID) return the zone
                        ret.append(b)
                        if len(ret) >= n:  # TODO continue with next key-value-pair when was found
                            return ret
        return ret

    def get_range(self, responses):
        # get range from a single SRG response if message is valid
        pass

    def parse_srg(self, responses):
        """
        Parse a list of SRG ranging responses of UWB beacons. For each message (if it is valid) return the positions,
        frame id and estimated ranges.
        :param responses: [String]: list of SRG responses from UWB beacons
        :return: [(Beacon, Float)]: list beacon-object and range pairs
        """
        ranges = []
        # iterate through each response and fill output lists if ranging messages are valid
        for r in responses:
            # split single message on spaces
            split = r.split(' ')
            # skip this message if it is invalid
            if len(split) != 6:
                print('Encountered invalid SRG response.')
                continue
            # get beacon object from EID
            beacon = self.get_beacon(split[1])
            # calculate range
            range = float(split[2]) / 1000.
            # append to output
            ranges.append((beacon, range))
        return ranges

    def trilaterate(self, ranges):
        """
        Estimate the position of the receiver by using trilateration with at least three UWB responses
        :param ranges: [([Float, Float, Float], Float)]: list of position and range pairs (obtained from parse_srg)
        :return: [Float, Float, Float]: 3D coordinates of estimated receiver location
        """
        # check whether enough points are given
        if len(ranges) < 3:
            return None
        # get points and distances from input, remember index of shortest distance
        points, distances = [], []
        for r in ranges:
            points.append(r[0])
            distances.append(r[1])
        # get initial guess TODO initial guess as beacon centroid, closest beacon or sth??
        initial_guess = [0, 0, 0]
        # minimize root mean square error to get estimated position
        res = minimize(self.mse, initial_guess, args=(points, distances), method='L-BFGS-B')
        return res.x

    @staticmethod
    def mse(x, points, distances):
        """
        Cost-function to use for position estimation. Minimize the squared error of the distance between a variable
        point x and each beacon and the measured UWB distance. mse = SUM(distance - dist(x, points)^2
        :param x: [Float, Float, Float]: vector with variable components to be optimized
        :param points: [[x, y, z], ...]: list of positions of the beacons used for UWB ranging
        :param distances: [Float]: estimated distances from UWB ranging of above beacons
        :return: Float: mean squared error for all beacon positions
        """
        mse = 0.
        for p, d in zip(points, distances):
            d_calc = math.sqrt((x[0]-p[0])**2 + (x[1]-p[1])**2 + (x[2]-p[2])**2)
            mse += (d_calc-d)**2
        return mse  # / len(points)


if __name__ == '__main__':
    # testing the class and its methods
    pos = PositioningPlus('/home/metratec/catkin_ws/src/ros_ips/config/zones.yml')
    dummy_pings = ['BCN 00124B00090593E6 -060', 'BCN 00124B00090593E6 -070', 'BCN 00124B00090593E6 -070',
                   'BCN 00124B00050CD41E -090', 'BCN 00124B00050CD41E -070', 'BCN 00124B00050CD41E -050',
                   'BCN 00124B00050CDC0A -070', 'BCN 00124B00050CDC0A -060', 'BCN 00124B00050CDC0A -090']
    beacon = pos.get_beacon('00124B00090593E6')
    zone = pos.get_zone(dummy_pings)
    top = pos.get_top_beacons(dummy_pings, 5)

    resp = ['SRG 00124B00050CDA71 01650 -076 -081 047\r', 'SRG 00124B00090593E6 04300 -076 -080 105\r',
            'SRG 00124B00050CD41E 00800 -079 -082 049\r']
    ranges = pos.parse_srg(resp)
    tril = pos.trilaterate(ranges)
    breakpoint = 0
