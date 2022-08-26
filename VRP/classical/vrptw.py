import numpy as np

"""
Solving VRPTW problem using Google OR-Tools.
"""


class VRPTW:
    """Solver for Vehicle Routing Problem with Time Windows - classical implementation
       based on Google OR-Tools, for testing purposes (reference solution)."""

    def __init__(
        self,
        n: int,
        m: int,
        dist: np.array,
        xc: np.array,
        yc: np.array,
        tw: np.array,
        time: np.array,
    ):
        self.n = n  # number of cities
        self.m = m  # number of vehicles
        self.dist = dist  # distance matrix (distances between cities)
        self.time = time  # travel times between locations
        self.xc = xc  # matrix of x coordinates for each city
        self.yc = yc  # matrix of y coordinates for each city
        self.tw = tw  # list of time windows (for each city)

    def solve(self):
        pass