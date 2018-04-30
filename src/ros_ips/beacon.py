#!/usr/bin/env python
"""
Use this class to store information about beacons that are placed in the environment and used for IPS tracking.
"""


class Beacon:
    """Class to store information about IPS beacons."""
    def __init__(self, eid, position, frame_id):
        """
        Initialize beacon object by passing its EID, position in the environment and name of coordinate frame.
        :param eid: String: unique EID of the beacon, used for unambiguous identification, given by the manufacturer
        :param position: [Float, Float, Float]: list of [x, y, z] coordinates of the beacon's position in 3D space
        :param frame_id: String: name of the coordinate frame the position is specified in
        """
        # store beacon characteristics
        self.eid = eid
        self.position = position
        self.frame_id = frame_id
