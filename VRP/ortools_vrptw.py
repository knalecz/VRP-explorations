#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8
# Copyright 2015 Tin Arm Engineering AB
# Copyright 2018 Google LLC
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# [START program]
"""Capacitated Vehicle Routing Problem with Time Windows (CVRPTW).

   This is a sample using the routing library python wrapper to solve a CVRPTW
   problem.
   A description of the problem can be found here:
   http://en.wikipedia.org/wiki/Vehicle_routing_problem.

   Distances are in meters and time in minutes.

   https://github.com/google/or-tools/blob/stable/ortools/constraint_solver/samples/cvrptw.py
"""

# [START import]
from functools import partial
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from vrptw_experiments import Initializer
# [END import]


# [START data_model]
def create_data_model():
    """Stores the data for the problem."""
    data = {}
    # Locations in block unit
    _locations = [
        (4, 4),  # depot
        (2, 0),
        (8, 0),  # locations to visit
        (0, 1),
        (1, 1),
        (5, 2),
        (7, 2),
        (3, 3),
        (6, 3),
        (5, 5),
        (8, 5),
        (1, 6),
        (2, 6),
        (3, 7),
        (6, 7),
        (0, 8),
        (7, 8),
    ]
    # Compute locations in meters using the block dimension defined as follow
    # Manhattan average block: 750ft x 264ft -> 228m x 80m
    # here we use: 114m x 80m city block
    # src: https://nyti.ms/2GDoRIe "NY Times: Know Your distance"
    data["locations"] = [(location[0] * 114, location[1] * 80) for location in _locations]
    data["num_locations"] = len(data["locations"])
    data["time_windows"] = [
        (0, 0),
        (75, 85),
        (75, 85),
        (60, 70),
        (45, 55),
        (0, 8),
        (50, 60),
        (0, 10),
        (10, 20),
        (0, 10),
        (75, 85),
        (85, 95),
        (5, 15),
        (15, 25),
        (10, 20),
        (45, 55),
        (30, 40),
    ]
    data["num_vehicles"] = 4
    data["vehicle_speed"] = 83  # Travel speed: 5km/h converted in m/min
    data["depot"] = 0
    return data
# [END data_model]


# [START data_model v. 2]
def create_random_data_model(clients_num, vehicles_num):
    """Stores the data for the problem."""

    initializer = Initializer(clients_num + 1, clients_num + 1, 0)
    xc, yc, dist, time, tw = initializer.generate_nodes_weight_matrix_time_windows()

    data = {}
    # Locations in block unit
    _locations = [
        (4, 4),  # depot
        (2, 0),
        (8, 0),  # locations to visit
        (0, 1),
        (1, 1),
        (5, 2),
        (7, 2),
        (3, 3),
        (6, 3),
        (5, 5),
        (8, 5),
        (1, 6),
        (2, 6),
        (3, 7),
        (6, 7),
        (0, 8),
        (7, 8),
    ]
    # Compute locations in meters using the block dimension defined as follow
    # Manhattan average block: 750ft x 264ft -> 228m x 80m
    # here we use: 114m x 80m city block
    # src: https://nyti.ms/2GDoRIe "NY Times: Know Your distance"
    data["locations"] = [(location[0] * 114, location[1] * 80) for location in _locations]
    data["num_locations"] = len(data["locations"])
    data["time_windows"] = [
        (0, 0),
        (75, 85),
        (75, 85),
        (60, 70),
        (45, 55),
        (0, 8),
        (50, 60),
        (0, 10),
        (10, 20),
        (0, 10),
        (75, 85),
        (85, 95),
        (5, 15),
        (15, 25),
        (10, 20),
        (45, 55),
        (30, 40),
    ]
    data["num_vehicles"] = vehicles_num
    data["vehicle_speed"] = 83  # Travel speed: 5km/h converted in m/min
    data["depot"] = 0
    return data
    # [END data_model]


#######################
# Problem Constraints #
#######################
def manhattan_distance(position_1, position_2):
    """Computes the Manhattan distance between two points"""
    return abs(position_1[0] - position_2[0]) + abs(position_1[1] - position_2[1])


def create_distance_evaluator(data):
    """Creates callback to return distance between points."""
    _distances = {}
    # precompute distance between location to have distance callback in O(1)
    for from_node in range(data["num_locations"]):
        _distances[from_node] = {}
        for to_node in range(data["num_locations"]):
            if from_node == to_node:
                _distances[from_node][to_node] = 0
            else:
                _distances[from_node][to_node] = manhattan_distance(
                    data["locations"][from_node], data["locations"][to_node]
                )

    def distance_evaluator(manager, from_node, to_node):
        """Returns the manhattan distance between the two nodes"""
        return _distances[manager.IndexToNode(from_node)][manager.IndexToNode(to_node)]

    return distance_evaluator


def create_time_evaluator(data):
    """Creates callback to get total times between locations."""

    def travel_time(data, from_node, to_node):
        """Gets the travel times between two locations."""
        if from_node == to_node:
            travel_time = 0
        else:
            travel_time = (
                manhattan_distance(
                    data["locations"][from_node], data["locations"][to_node]
                )
                / data["vehicle_speed"]
            )
        return travel_time

    _total_time = {}
    # precompute total time to have time callback in O(1)
    for from_node in range(data["num_locations"]):
        _total_time[from_node] = {}
        for to_node in range(data["num_locations"]):
            if from_node == to_node:
                _total_time[from_node][to_node] = 0
            else:
                _total_time[from_node][to_node] = int(travel_time(data, from_node, to_node))

    def time_evaluator(manager, from_node, to_node):
        """Returns the total time between the two nodes"""
        return _total_time[manager.IndexToNode(from_node)][manager.IndexToNode(to_node)]

    return time_evaluator


def add_time_window_constraints(routing, manager, data, time_evaluator_index):
    """Add Global Span constraint"""
    time = "Time"
    horizon = 120
    routing.AddDimension(
        time_evaluator_index,
        horizon,  # allow waiting time
        horizon,  # maximum time per vehicle
        False,  # don't force start cumul to zero since we are giving TW to start nodes
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location except depot
    # and 'copy' the slack var in the solution object (aka Assignment) to print it
    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx == 0:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        routing.AddToAssignment(time_dimension.SlackVar(index))
    # Add time window constraints for each vehicle start node
    # and 'copy' the slack var in the solution object (aka Assignment) to print it
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            data["time_windows"][0][0], data["time_windows"][0][1]
        )
        routing.AddToAssignment(time_dimension.SlackVar(index))
        # Warning: Slack var is not defined for vehicle's end node
        # routing.AddToAssignment(time_dimension.SlackVar(self.routing.End(vehicle_id)))


# [START solution_printer]
def print_solution(manager, routing, assignment):  # pylint:disable=too-many-locals
    """Prints assignment on console"""
    print(f"Objective: {assignment.ObjectiveValue()}")
    time_dimension = routing.GetDimensionOrDie("Time")
    total_distance = 0
    total_time = 0
    for vehicle_id in range(manager.GetNumberOfVehicles()):
        index = routing.Start(vehicle_id)
        plan_output = "Route for vehicle {}:\n".format(vehicle_id)
        distance = 0
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            slack_var = time_dimension.SlackVar(index)
            plan_output += " {0} Time({1},{2}) Slack({3},{4}) ->".format(
                manager.IndexToNode(index),
                assignment.Min(time_var),
                assignment.Max(time_var),
                assignment.Min(slack_var),
                assignment.Max(slack_var),
            )
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        time_var = time_dimension.CumulVar(index)
        slack_var = time_dimension.SlackVar(index)
        plan_output += " {0} Time({1},{2})\n".format(
            manager.IndexToNode(index),
            assignment.Min(time_var),
            assignment.Max(time_var),
        )
        plan_output += "Distance of the route: {0}m\n".format(distance)
        plan_output += "Time of the route: {}\n".format(assignment.Value(time_var))
        print(plan_output)
        total_distance += distance
        total_time += assignment.Value(time_var)
    print("Total Distance of all routes: {0}m".format(total_distance))
    print("Total Time of all routes: {0}min".format(total_time))
    # [END solution_printer]


def main():
    """Solve the Capacitated VRP with time windows."""
    # Instantiate the data problem.
    # [START data]
    data = create_data_model()
    # [END data]

    # Create the routing index manager.
    # [START index_manager]
    manager = pywrapcp.RoutingIndexManager(
        data["num_locations"], data["num_vehicles"], data["depot"]
    )
    # [END index_manager]

    # Create Routing Model.
    # [START routing_model]
    routing = pywrapcp.RoutingModel(manager)
    # [END routing_model]

    # Define weight of each edge.
    # [START transit_callback]
    distance_evaluator_index = routing.RegisterTransitCallback(
        partial(create_distance_evaluator(data), manager)
    )
    # [END transit_callback]

    # Define cost of each arc.
    # [START arc_cost]
    routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator_index)
    # [END arc_cost]

    # Add Time Window constraint.
    # [START time_constraint]
    time_evaluator_index = routing.RegisterTransitCallback(
        partial(create_time_evaluator(data), manager)
    )
    add_time_window_constraints(routing, manager, data, time_evaluator_index)
    # [END time_constraint]

    # Setting first solution heuristic (cheapest addition).
    # [START parameters]
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(2)
    search_parameters.log_search = True
    # [END parameters]

    # Solve the problem.
    # [START solve]
    solution = routing.SolveWithParameters(search_parameters)
    # [END solve]

    # Print solution on console.
    # [START print_solution]
    if solution:
        print_solution(manager, routing, solution)
    else:
        print("No solution found!")
    # [END print_solution]


if __name__ == "__main__":
    main()
# [END program]
