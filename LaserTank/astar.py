#!/usr/bin/python
import sys
import queue
import numpy as np
import time
import heapq
from laser_tank import LaserTankMap

"""
Template file for you to implement your solution to Assignment 1.

COMP3702 2020 Assignment 1 Support Code
"""



# Code for any classes or functions you need can go here.
class Node:

    def __init__(self, lasertank, cost, path):
        self.lasertank = lasertank
        self.cost = cost
        self.total_cost = cost
        self.path = path

    def __eq__(self, other):
        return self.lasertank.grid_data == other.lasertank.grid_data

    def __lt__(self, other):

        return self.total_cost < other.total_cost

    def deep_copy(self):
        new_tank = LaserTankMap(self.lasertank.x_size, self.lasertank.y_size, self.lasertank.grid_data, self.lasertank.player_x, self.lasertank.player_y, self.lasertank.player_heading)
        copy_grid = []
        for row in self.lasertank.grid_data:
            copy_grid.append(row.copy())

        new_tank.grid_data = copy_grid

        copy = Node(new_tank, self.cost, self.path)
        return copy

    def estimate_cost(self,goal_x,goal_y):
        estimate_x = abs(goal_x - self.lasertank.player_x)
        estimate_y = abs(goal_y - self.lasertank.player_y)
        estimate_cost = estimate_x + estimate_y
        #test_estimate = max(estimate_y,estimate_x)
        return estimate_cost
#

def distance(x,y,x1,y1):
    distance = np.sqrt((x-x1)**2+(y-y1)**2)
    return distance






#


def write_output_file(filename, actions):
    """
    Write a list of actions to an output file. You should use this method to write your output file.
    :param filename: name of output file
    :param actions: list of actions where is action is in LaserTankMap.MOVES
    """
    f = open(filename, 'w')
    for i in range(len(actions)):
        f.write(str(actions[i]))
        if i < len(actions) - 1:
            f.write(',')
    f.write('\n')
    f.close()


def main():
    # input_file = arglist[0]
    # output_file = arglist[1]
    input_file = "testcases/t1_bridgeport.txt"
    # Read the input testcase file
    game_map = LaserTankMap.process_input_file(input_file)

    # show game map
    #game_map.render()
    actions = []

    # get the coordinate of goal
    for i in range(game_map.x_size):
        for j in range(game_map.y_size):
            if game_map.grid_data[j][i] == "F":
                goal_x = i
                goal_y = j

    #==========================================
    #   get the teleport
    pos = 0
    teleportx = {pos: 0}
    teleporty = {pos: 0}
    exist_tele = 0

    for i in range(game_map.x_size):
        for j in range(game_map.y_size):
            if game_map.grid_data[j][i] == "T":
                teleportx[pos] = i
                teleporty[pos] = j
                exist_tele = 1
                pos += 1






    # assignment test variable
    #number_node_create = 0
    #number_node_fringe = 0

    # ===========================================

    # count the start time
    start_time = time.time()

    # 4 actions
    actionset = ["f", "r", "l", "s"]

    # record the start node
    start = Node(lasertank=game_map, cost=0, path="")

    # test the estimate cost
    #estimate_cost = start.estimate_cost(goal_x,goal_y)
    #print(estimate_cost)
    #print(start.lasertank.player_x,start.lasertank.player_y)

    # the set of explored
    id = 0
    explored = {id:(start.lasertank.player_x,start.lasertank.player_y,start.lasertank.player_heading,start.lasertank.grid_data)}
    #map_explored = {id: start.lasertank.grid_data}
    # set the frontier queue
    heapq.heappush(actions,start)

    while len(actions) > 0:
        #heapq.heapify(actions)
        current_node = heapq.heappop(actions)

        # check if arrive the goal
        if current_node.lasertank.is_finished():
            end_time = time.time()
            run_time = end_time - start_time
            print("Find the Solution successfully!")
            print("the map is: ", input_file)

            # ===========================================
            # print("the number of Node generated: ", number_node_create)
            # print("the number of Node in fringe: ",len(actions))
            # print("the number of Node on explored: ", id)

            # ===========================================
            actions = current_node.path
            print("The path is: " + str(actions))
            print("The Steps are: ", len(actions))
            print("The time is: " + str(run_time))
            break

        # add the current node to explored
        id += 1
        explored[id] = (current_node.lasertank.player_x,current_node.lasertank.player_y,current_node.lasertank.player_heading,current_node.lasertank.grid_data)
        #map_explored[id] = (current_node.lasertank.grid_data)

        # serach for children
        for move in actionset:

            node_copy = current_node.deep_copy()

            status = node_copy.lasertank.apply_move(move)

            child_path = node_copy.path + move
            child_cost = node_copy.cost + 1

            if status == 0:     # SUCCESS

                # check node if existed in explored
                if (node_copy.lasertank.player_x,node_copy.lasertank.player_y,node_copy.lasertank.player_heading,node_copy.lasertank.grid_data) in explored.values():
                    continue
                else:
                    # add in queue
                    """if (node_copy.lasertank.grid_data) not in map_explored.values():
                        child_cost -= 1"""
                    node_copy.path = child_path
                    node_copy.cost = child_cost
                    total_cost = node_copy.cost + node_copy.estimate_cost(goal_x,goal_y)
                    if exist_tele == 1:
                        total_cost = min(total_cost, (distance(node_copy.lasertank.player_x,node_copy.lasertank.player_y,teleportx[0],teleporty[0])+distance(teleportx[1],teleporty[1],goal_x,goal_y)),(distance(node_copy.lasertank.player_x,node_copy.lasertank.player_y,teleportx[1],teleporty[1])+distance(teleportx[0],teleporty[0],goal_x,goal_y)))

                    node_copy.total_cost = total_cost
                    heapq.heappush(actions,node_copy)

                    # +++++++++test++++++++++++====
                    #number_node_create += 1

                    # +++++++++++++++++++++++++++++

                    # check the frontier queue
                    #print(node_copy.path)







    # Write the solution to the output file
    # write_output_file(output_file, actions)


if __name__ == '__main__':
    main()

