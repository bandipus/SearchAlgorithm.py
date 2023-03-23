# This file contains all the required routines to make an A* search algorithm.
#
__authors__ = '1636012'
__group__ = 'DM.18'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Curs 2022 - 2023
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
from utils import *
import os
import math
import copy


def expand(path, map):
    """
     It expands a SINGLE station and returns the list of class Path.
     Format of the parameter is:
        Args:
            path (object of Path class): Specific path to be expanded
            map (object of Map class):: All the information needed to expand the node
        Returns:
            path_list (list): List of paths that are connected to the given path.
    """
    
    path_list = []
    connections = map.connections[path.last]
    
    for station in connections:
        new_path = Path(list(path.route))
        new_path.add_route(station)
        path_list.append(new_path)
    
    return path_list
    
def remove_cycles(path_list):
    """
     It removes from path_list the set of paths that include some cycles in their path.
     Format of the parameter is:
        Args:
            path_list (LIST of Path Class): Expanded paths
        Returns:
            path_list (list): Expanded paths without cycles.
    """
    
    unique_values = []
    new_path_list = []
    
    for path in path_list:
        iterable_list = list(path.route)
        for stop in iterable_list:
            if stop not in unique_values:
                unique_values.append(stop)
            else:
                break
        if len(unique_values) == len(iterable_list):
            new_list = Path(iterable_list)
            new_path_list.append(new_list)
        unique_values.clear()
    
    return new_path_list

def insert_depth_first_search(expand_paths, list_of_path):
    """
     expand_paths is inserted to the list_of_path according to DEPTH FIRST SEARCH algorithm
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            list_of_path (LIST of Path Class): The paths to be visited
        Returns:
            list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    
    for paths in list_of_path:
        expand_paths.append(paths)
    
    return expand_paths

def depth_first_search(origin_id, destination_id, map):
    """
     Depth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): the route that goes from origin_id to destination_id
    """
    
    origin = Path([origin_id])
    
    list_of_path = [origin]
    
    while list_of_path:
        C = list_of_path.pop(0)
        
        if C.last == destination_id:
            return C
        
        E = expand(C,map)
        E = remove_cycles(E)
        list_of_path = insert_depth_first_search(E, list_of_path)
        
    return "No existeix Solucio"

def insert_breadth_first_search(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to BREADTH FIRST SEARCH algorithm
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    
    for paths in expand_paths:
        list_of_path.append(paths)
    
    return list_of_path

def breadth_first_search(origin_id, destination_id, map):
    """
     Breadth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    
    origin = Path([origin_id])
    
    list_of_path = [origin]
    
    while list_of_path:
        C = list_of_path.pop(0)
            
        if C.last == destination_id:
            return C
        
        E = expand(C,map)
        E = remove_cycles(E)
        list_of_path = insert_breadth_first_search(E, list_of_path)
        
    return "No existeix Solucio"

def calculate_cost(expand_paths, map, type_preference=0):
    """
         Calculate the cost according to type preference
         Format of the parameter is:
            Args:
                expand_paths (LIST of Paths Class): Expanded paths
                map (object of Map class): All the map information
                type_preference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
            Returns:
                expand_paths (LIST of Paths): Expanded path with updated cost
    """
    
    for path in expand_paths:
        if type_preference == 0:
            path.update_g(len(path.route) - 1)

        elif type_preference == 1:
            list_of_path = path.route
            for i in range(len(list_of_path) - 1):
                new_station = list_of_path[i + 1]
                last_station = list_of_path[i]
                time = map.connections[last_station][new_station]
                path.update_g(time)

        elif type_preference == 2:
            list_of_path = path.route
            for i in range(len(list_of_path) - 1):
                new_station = list_of_path[i + 1]
                last_station = list_of_path[i]
                if map.stations[last_station]['name'] != map.stations[new_station]['name']:
                    distance = map.connections[last_station][new_station] * map.stations[last_station]['velocity']
                    path.update_g(distance)

        elif type_preference == 3:
            list_of_path = path.route
            for i in range(len(list_of_path) - 1):
                new_station_line = map.stations[list_of_path[i + 1]]['line']
                last_station_line = map.stations[list_of_path[i]]['line']
                if last_station_line != new_station_line:
                    path.update_g(1)
    
    return expand_paths

def insert_cost(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to COST VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to cost
    """
    list_of_path = list_of_path + expand_paths
    list_of_path.sort(key=lambda x: x.g)
    return list_of_path

def uniform_cost_search(origin_id, destination_id, map, type_preference=0):
    """
     Uniform Cost Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    
    origin = Path([origin_id])
    
    list_of_path = [origin]
    
    while list_of_path:
        C = list_of_path.pop(0)
            
        if C.last == destination_id:
            return C
        
        E = expand(C,map)
        E = remove_cycles(E)
        E = calculate_cost(E,map,type_preference)
        list_of_path = insert_cost(E, list_of_path)
        
    return "No existeix Solucio"

def calculate_heuristics(expand_paths, map, destination_id, type_preference=0):
    """
     Calculate and UPDATE the heuristics of a path according to type preference
     WARNING: In calculate_cost, we didn't update the cost of the path inside the function
              for the reasons which will be clear when you code Astar (HINT: check remove_redundant_paths() function).
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            map (object of Map class): All the map information
            destination_id (int): Final station id
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            expand_paths (LIST of Path Class): Expanded paths with updated heuristics
    """

    for path in expand_paths:
        if type_preference == 0:
            if map.stations[path.last] != map.stations[destination_id]:
                path.update_h(1)
            else:
                path.update_h(0)

        elif type_preference == 1:
            last_station = map.stations[path.last]
            dest_station = map.stations[destination_id]
            distance = math.sqrt((last_station['x'] - dest_station['x']) ** 2 + (last_station['y'] - dest_station['y']) ** 2)
            maximum_velocity = max(map.velocity.values())
            time = distance / maximum_velocity
            path.update_h(time)

        elif type_preference == 2:
            last_station = map.stations[path.last]
            dest_station = map.stations[destination_id]
            distance = math.sqrt((last_station['x'] - dest_station['x']) ** 2 + (last_station['y'] - dest_station['y']) ** 2)
            path.update_h(distance)

        elif type_preference == 3:
            if map.stations[path.last]['line'] != map.stations[destination_id]['line']:
                path.update_h(1)
            else:
                path.update_h(0)

    return expand_paths

def update_f(expand_paths):
    """
      Update the f of a path
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
         Returns:
             expand_paths (LIST of Path Class): Expanded paths with updated costs
    """
    for path in expand_paths:
        path.update_f()
    
    return expand_paths

def remove_redundant_paths(expand_paths, list_of_path, visited_stations_cost):
    """
      It removes the Redundant Paths. They are not optimal solution!
      If a station is visited and have a lower g-cost at this moment, we should remove this path.
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
             list_of_path (LIST of Path Class): All the paths to be expanded
             visited_stations_cost (dict): All visited stations cost
         Returns:
             new_paths (LIST of Path Class): Expanded paths without redundant paths
             list_of_path (LIST of Path Class): list_of_path without redundant paths
             visited_stations_cost (dict): Updated visited stations cost
    """
    new_paths = []
    for path in expand_paths:
        if path.last in visited_stations_cost:
            if path.g >= visited_stations_cost[path.last]:
                continue
            visited_stations_cost[path.last] = path.g
            new_paths.append(path)

            list_of_path = [p for p in list_of_path if path.last not in p.route]

    return new_paths, list_of_path, visited_stations_cost

def insert_cost_f(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to f VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to f
    """
    list_of_path = list_of_path + expand_paths
    list_of_path.sort(key=lambda x: x.f)
    return list_of_path

def coord2station(coord, map):
    """
        From coordinates, it searches the closest stations.
        Format of the parameter is:
        Args:
            coord (list):  Two REAL values, which refer to the coordinates of a point in the city.
            map (object of Map class): All the map information
        Returns:
            possible_origins (list): List of the Indexes of stations, which corresponds to the closest station
    """
    
    station_coords = []
    distances = []
    possible_origins = []
    
    for id, station in map.stations.items():
        x = station["x"]
        y = station["y"]
        station_coords.append([id,x,y])
        
    for i in range(len(station_coords)):
        distance = euclidean_dist(coord, [station_coords[i][1],station_coords[i][2]])
        distances.append([station_coords[i][0], distance])
    
    # Ordena les distancies de menor a major
    sorted_distances = sorted(distances, key=lambda x: x[1])
    
    minimum_distance = sorted_distances[0][1]
    i = 0
    
    while sorted_distances[i][1] == minimum_distance:
        possible_origins.append(sorted_distances[i][0])
        i += 1
        
    return possible_origins

def Astar(origin_id, destination_id, map, type_preference=0):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    origin = Path([origin_id])
    
    list_of_path = [origin]

    TCP = {}
    
    while list_of_path:
        C = list_of_path.pop(0)
            
        if C.last == destination_id:
            return C
        
        E = expand(C,map)
        E = remove_cycles(E)
        E = calculate_heuristics(E, map, destination_id, type_preference)
        E = calculate_cost(E,map,type_preference)
        update_f(E)
        E,list_of_path,TCP = remove_redundant_paths(E,list_of_path,TCP)
        list_of_path = insert_cost_f(E, list_of_path)
        
    return "No existeix Solucio"
