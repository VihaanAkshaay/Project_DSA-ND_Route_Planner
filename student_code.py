import heapq
from math import pow, sqrt
from collections import namedtuple
Cost = namedtuple('Cost',['total','journey','to_goal']) #(float,float)
Path = namedtuple('Path',['cost','intersections','previous','frontier']) #(Cost,[int,int,.....],int ,int)

'''This function defines the displacement between two towns which 
is used for the straight line distance (ie. h) that we add to the 
cost function that we aim to minimize'''


def displacement(point1, point2):
    return sqrt(pow((point1[0] - point2[0]), 2) + pow((point1[1] - point2[1]), 2))

def update_path(M,path,new_frontier,goal):
    traversed_distance = displacement(M.intersections[path.frontier],M.intersections[new_frontier])
    new_path_cost_journey = path.cost.journey + traversed_distance
    new_path_cost_to_goal = displacement(M.intersections[new_frontier],M.intersections[goal])
    new_path_cost_total = new_path_cost_journey + new_path_cost_to_goal
    new_path_intersections = path.intersections + [new_frontier]

    new_path = Path(Cost(new_path_cost_total,new_path_cost_journey,new_path_cost_to_goal),new_path_intersections,path.frontier,new_frontier)
    return new_path

def shortest_path(M, start, goal):
    paths = list()
    path_goal_min_val = float('inf')
    path_goal_min = None

    #Checking if it is already in goal
    if start == goal:
        return [start]

    #initializing paths

    goal_initial_distance = displacement(M.intersections[start],M.intersections[goal])
    path = Path(Cost(goal_initial_distance,0,goal_initial_distance),[start],start,start)
    heapq.heappush(paths,path)

    while len(paths) >= 1:
        nearest_frontier_path = heapq.heappop(paths)
        for neighbour_road in M.roads[nearest_frontier_path.frontier]:
            if neighbour_road == nearest_frontier_path.previous:
                continue
            else:
                new_path = update_path(M,path= nearest_frontier_path,new_frontier = neighbour_road,goal = goal)

                if neighbour_road == goal: #Reached destination with a path
                    if new_path.cost.total < path_goal_min_val:
                        path_goal_min_val = new_path.cost.total
                        path_goal_min = new_path.intersections
                    else: #Reached destination with a higher cost is discaraded
                        pass
                else:
                    if path_goal_min is not None: #Already found the goal in a different path
                        if new_path.cost.total >= path_goal_min_val:
                            pass
                        else: #Cheaper path keep exploring
                            heapq.heappush(paths,new_path)
                    else: #The goal isn't found yet, continue exploring
                        heapq.heappush(paths,new_path)

    if path_goal_min is not None:
        return path_goal_min



    print("shortest path called")
    return