#!/usr/bin/env python
"""
Zone class, imports Beacon
"""


class Zone:
    def __init__(self, name, frame_id, threshold, polygon, beacons):
        # store zone characteristics
        self.name = name
        self.frame_id = frame_id
        self.threshold = threshold
        self.polygon = polygon
        self.beacons = beacons
