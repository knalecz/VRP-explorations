# import numpy as np
# import matplotlib as mpl
# import matplotlib.pyplot as plt
# import networkx as nx

"""
Solving a custom formulation (Full Qubo Solver) of VRPTW using CP-SAT solver.
"""

MAX_EARLIEST_TIME = 5       # max. "earliest arrival time" (lower bound for time window)
MAX_TIME_WINDOW_RANGE = 30  # the largest possible time window size


class FQSTW:
    """Full Qubo Solver for Vehicle Routing Problem with Time Windows - classical implementation"""

    def __init__(self, n, m, cost, xc, yc, tw):
        self.n = n        # number of cities
        self.m = m        # number of vehicles
        self.cost = cost  # cost/weight matrix (distances between cities)
        self.xc = xc      # matrix of x coordinates for each city
        self.yc = yc      # matrix of y coordinates for each city
        self.tw = tw      # list of time windows (for each client/city)
        self.A_max = 30   # TODO wyliczyć na podstawie tw
        self.W_max = 30   # TODO wyliczyć na podstawie tw
        self.sol = None   # solution of VRPTW problem
        self.model = None
        self.formulate()
