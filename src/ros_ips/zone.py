#!/usr/bin/env python
"""
Zone class, imports Beacon
"""


class Zone:
    def __init__(self, name, frame_id, polygon, beacons):
        # store zone characteristics
        self.name = name
        self.frame_id = frame_id
        self.polygon = polygon
        self.beacons = beacons
