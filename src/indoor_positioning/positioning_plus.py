#!/usr/bin/env python
"""
Use this class to perform indoor positioning with the metraTec IPS+ system. The position of a receiver is evaluated by
UWB ranging of RF beacons placed in the environment. Requires a YAML file with the configuration of beacon positions.

Usage:
Initialize the class by passing the directory of the config file. Then collect a number of beacon responses to ranging
requests (SRG) and use the parse_srg() function to get a list of beacon objects and their respective ranges from a list
of SRG responses.
Use this list as input for the trilaterate() function to estimate the position of the receiver in 3D space.

A few examples can be found in the main function below.
"""

import math
import operator
from scipy.optimize import minimize
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from positioning import Positioning


class PositioningPlus(Positioning):
    """Class to extend the functionality of the Positioning class with UWB ranging capabilities."""

    # options for minimization function used for trilateration
    minimization_params = dict(method='TNC',  # L-BFGS-B, TNC or SLSQP
                               options={'disp': True})

    def __init__(self, config_dir, min_beacons=4, max_z=None, dilation=0.0):
        """
        Initialize the class with zone and beacon definitions from YAML configuration file.
        :param config_dir: String: Directory of the YAML config file
        :param min_beacons: Int: Minimum number of beacons to use for trilateration
        :param max_z: Float: Maximum z position the receiver can have after ranging
        """
        # initialize the positioning class that this class inherits
        Positioning.__init__(self, config_dir)

        # minimum number of beacons (and ranges) to use for position estimation
        self.min_beacons = min_beacons

        # store EIDs of configured beacons for easy access
        self.eids = self.get_defined_beacons()

        # maximum z value the position estimate can have (used as initialization and upper bound for trilateration)
        self.max_z = max_z

        # dilation for the polygon check
        self.dilation = dilation

    def get_top_beacons(self, pings, n):
        """
        Get the top n beacons in terms of RSSI values from a list of beacon pings.
        :param pings: [String]: list of beacon pings
        :param n: Int: Number of top beacons to return
        :return: [Beacon]: list of beacon objects ordered according to RSSI values
        """
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
                print('Encountered invalid SRG response: {}'.format(r))
                continue
            # get beacon object from EID
            beacon = self.get_beacon(split[1])
            # calculate range
            range = float(split[2]) / 1000.
            # append to output
            ranges.append((beacon, range))
        return ranges

    def in_range(self, pings, threshold=None):
        """
        Get a list of beacons that are in range and are configured in the configuration YAML file from a list of
        beacon pings recorded over a short duration. The returned beacons (EIDs) can be used for UWB ranging
        :param pings: [String]: List of BCN pings
        :param threshold: Int: Minimum RSSI value a beacon should have to be returned.
        :return: [String]: List of EIDs that can be used for ranging
        """
        eids = []
        # get unique list of beacon EIDs from list of pings and their average RSSI value
        mean = self.get_mean(pings)
        # sort mean dictionary according to RSSI values
        srtd = sorted(mean.items(), key=operator.itemgetter(1), reverse=True)
        for s in srtd:
            # Don't append this beacon to list if the RSSI value is too low
            if threshold is not None and s[1] < threshold:
                break
            # check if beacon is configured in yml file
            if s[0] in self.eids:
                eids.append(s[0])
        return eids

    def trilaterate(self, ranges):
        """
        Estimate the position of the receiver by using trilateration with at least three UWB responses
        :param ranges: [([Float, Float, Float], Float)]: list of position and range pairs (obtained from parse_srg) or
                       [(Beacon, Float)]: list of beacon object and range pairs (output from parse_srg())
        :return: [Float, Float, Float]: 3D coordinates of estimated receiver location
        """
        # check whether enough points are given
        if len(ranges) < self.min_beacons:
            print('Not enough beacons for position estimation ({}/{})'.format(len(ranges), self.min_beacons))
            return False, None
        print('Using {} beacons for trilateration.'.format(len(ranges)))
        # get points and distances from input and compute sum of distances
        points, poly_points, distances, dsum = [], [], [], 0
        for r in ranges:
            # get position of beacon directly from input list or alternatively from Beacon object
            p = r[0].position if r[0].__class__.__name__ == 'Beacon' else r[0]
            points.append(p)
            poly_points.append((p[0], p[1]))
            distances.append(r[1])
            dsum += r[1]
        # get weighted centroid
        initial = [0, 0, 0]
        for p, d in zip(points, distances):
            weight = 1. - d / dsum
            initial = [initial[0] + p[0] * weight, initial[1] + p[1] * weight, initial[2] + p[2] * weight]
        initial = [initial[0] / len(ranges), initial[1] / len(ranges), initial[2] / len(ranges)]
        if self.max_z is not None and initial[2] > self.max_z:
            initial[2] = self.max_z
        # minimize root mean square error to get estimated position
        print('guess: ', initial)
        # impose upper bound on z coordinate
        bnds = ((None, None), (None, None), (None, self.max_z)) if self.max_z is not None else None
        res = minimize(self.mse, initial, args=(points, distances), bounds=bnds, **self.minimization_params)
        print('final: ', res.x)
        # check if result is within bounds
        poly = Polygon(poly_points)
        return poly.convex_hull.buffer(self.dilation).contains(Point(res.x[0], res.x[1])), res.x

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
    """Testing the class and its methods."""
    # initialize class
    pos = PositioningPlus('/home/metratec/catkin_ws/src/indoor_positioning/config/zones.yml')
    # create a list of pings that would usually be collected from the receiver and stored in a buffer
    dummy_pings = ['BCN 00124B00090593E6 -090', 'BCN 00124B00090593E6 -090', 'BCN 00124B00090593E6 -090',
                   'BCN 00124B00050CD41E -090', 'BCN 00124B00050CD41E -070', 'BCN 00124B00050CD41E -050',
                   'BCN 00124B00050CDC0A -070', 'BCN 00124B00050CDC0A -060', 'BCN 00124B00050CDC0A -090']
    # get the beacon object with a specified EID
    # Note: the above beacons have to be defined in the config file for this to work
    beacon = pos.get_beacon('00124B00090593E6')
    # get the current zone the receiver is in based on the passed list of pings
    zone = pos.get_zone(dummy_pings)

    # create a list of UWB ranging responses that would usually be collected from a receiver and stored in a buffer
    resp = ['SRG 00124B00050CDA71 01650 -076 -081 047\r', 'SRG 00124B00090593E6 04300 -076 -080 105\r',
            'SRG 00124B00050CD41E 00800 -079 -082 049\r']
    # get beacons in range for ranging
    in_range = pos.in_range(dummy_pings)
    # get beacon objects and respective ranges from a list of UWB ranging responses
    ranges = pos.parse_srg(in_range)
    # use trilateration (with at least 3 points) to estimate the receiver location from beacon positions and ranges
    tril = pos.trilaterate(ranges)
    # break here to investigate variables etc.
    breakpoint = 0
