#!/usr/bin/env python
"""
Use this class to store information about zones for the metraTec IPS tracking system and zone location.
"""


class Zone:
    """Class to store information about zones defined for use with the metraTec IPS tracking system."""
    def __init__(self, name, frame_id, threshold, polygon, beacons):
        """
        Initialize the zone object by passing configuration and a list of beacons belonging to the zone.
        :param name: String: name of the zone
        :param frame_id: String: coordinate frame the zone is specified in
        :param threshold: Int: minimum RSSI value the a beacon must have to assign the receiver position to this zone
        :param polygon: [[Float, Float, Float]]: list of ordered [x, y, z] coordinates spanning the zone
        :param beacons: [Beacon]: List of beacon objects that belong to this zone
        """
        # store zone characteristics
        self.name = name
        self.frame_id = frame_id
        self.threshold = threshold
        self.polygon = polygon
        self.beacons = beacons
