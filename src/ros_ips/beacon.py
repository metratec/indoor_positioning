#!/usr/bin/env python
"""
Beacon class
"""


class Beacon:
    def __init__(self, eid, position, frame_id):
        # store beacon characteristics
        self.eid = eid
        self.position = position
        self.frame_id = frame_id