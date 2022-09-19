import numpy as np

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


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

    def __prepare_data(self):
        data = {}
        data["distance_matrix"] = self.dist.tolist()
        data["time_matrix"] = self.time.tolist()
        data["time_windows"] = self.tw.tolist()
        data["num_vehicles"] = self.m
        data["depot"] = 0

        return data

    def __print_solution(self, manager, routing, solution):
        """Prints solution on console."""
        print(f"Objective: {solution.ObjectiveValue()}")
        time_dimension = routing.GetDimensionOrDie("Time")
        total_distance = 0
        total_time = 0
        for vehicle_id in range(manager.GetNumberOfVehicles()):
            index = routing.Start(vehicle_id)
            plan_output = "Route for vehicle {}:\n".format(vehicle_id)
            distance = 0
            while not routing.IsEnd(index):
                time_var = time_dimension.CumulVar(index)
                plan_output += "{0} Time({1},{2}) -> ".format(
                    manager.IndexToNode(index),
                    solution.Min(time_var),
                    solution.Max(time_var),
                )

                previous_index = index
                index = solution.Value(routing.NextVar(index))
                distance += routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id
                )
            time_var = time_dimension.CumulVar(index)
            plan_output += "{0} Time({1},{2})\n".format(
                manager.IndexToNode(index),
                solution.Min(time_var),
                solution.Max(time_var),
            )
            plan_output += "Distance of the route: {0}m\n".format(distance)
            plan_output += "Time of the route: {}min\n".format(solution.Min(time_var))
            print(plan_output)
            total_distance += distance
            total_time += solution.Min(time_var)
        print("Total Distance of all routes: {0}m".format(total_distance))
        print("Total time of all routes: {}min".format(total_time))

    def solve(self):
        data = self.__prepare_data()
        manager = pywrapcp.RoutingIndexManager(
            len(data["time_matrix"]), data["num_vehicles"], data["depot"]
        )
        routing = pywrapcp.RoutingModel(manager)

        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data["distance_matrix"][from_node][to_node]

        distance_evaluator_index = routing.RegisterTransitCallback(distance_callback)

        routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator_index)

        def time_callback(from_index, to_index):
            """Returns the travel time between the two nodes."""
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data["time_matrix"][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(time_callback)

        # time windows constraint
        time = "Time"
        routing.AddDimension(
            transit_callback_index,
            30,  # allow waiting time
            30,  # maximum time per vehicle
            False,  # Don't force start cumul to zero.
            time,
        )
        time_dimension = routing.GetDimensionOrDie(time)

        for location_idx, time_window in enumerate(data["time_windows"]):
            if location_idx == data["depot"]:
                continue
            index = manager.NodeToIndex(location_idx)
            time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

        depot_idx = data["depot"]
        for vehicle_id in range(data["num_vehicles"]):
            index = routing.Start(vehicle_id)
            time_dimension.CumulVar(index).SetRange(
                data["time_windows"][depot_idx][0], data["time_windows"][depot_idx][1]
            )

        for i in range(data["num_vehicles"]):
            routing.AddVariableMinimizedByFinalizer(
                time_dimension.CumulVar(routing.Start(i))
            )
            routing.AddVariableMinimizedByFinalizer(
                time_dimension.CumulVar(routing.End(i))
            )

        # setting solution heuristic
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )

        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            self.__print_solution(manager, routing, solution)

            return dict({"min_cost": solution.ObjectiveValue()})

        print("No solution found!")

    def visualize(self):
        """Visualize solution"""
        pass
