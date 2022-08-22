import numpy as np
import networkx as nx
from vrpy import VehicleRoutingProblem

"""
Solving VRPTW problem using VRPy library.
"""


class VRPTW:
    """Solver for Vehicle Routing Problem with Time Windows - classical implementation
       based on VRPy (https://vrpy.readthedocs.io/en/latest/index.html) library,
       for testing purposes (reference solution)."""

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

    def __transform(self, mtx: np.array):
        n = mtx.shape[0]
        new_mtx = np.zeros([n + 1, n + 1])
        for i in range(1, n):
            new_mtx[0, i] = new_mtx[i, n] = mtx[0, i]
            for j in range(i + 1, n):
                new_mtx[i, j] = new_mtx[j, i] = mtx[i, j]

        return new_mtx

    def solve(self):
        tw_lower = {0: 0}
        tw_upper = {}
        for i in range(1, self.n + 1):
            tw_lower[i] = self.tw[i][0].item()
            tw_upper[i] = self.tw[i][1].item()

        distance_matrix = self.__transform(self.dist)
        time_matrix = self.__transform(self.time)
        
        G_d = nx.from_numpy_matrix(np.array(distance_matrix, dtype=[("cost", int)]), create_using=nx.DiGraph())

        G_t = nx.from_numpy_matrix(np.array(time_matrix, dtype=[("time", int)]), create_using=nx.DiGraph())

        G = nx.compose(G_d, G_t)

        nx.set_node_attributes(G, values=tw_lower, name="lower")
        nx.set_node_attributes(G, values=tw_upper, name="upper")

        G = nx.relabel_nodes(G, {0: "Source", self.n + 1: "Sink"})

        # TODO odkomentować
        # prob = VehicleRoutingProblem(G, time_windows=True, num_vehicles=self.m)
        prob = VehicleRoutingProblem(G, time_windows=True)
        prob.solve()

        return dict({
            'overall_solution_cost': prob.best_value,
            'best_routes': prob.best_routes,
            'arrival_time': prob.arrival_time
        })
