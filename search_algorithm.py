import pygame
import graphUI
from node_color import white, yellow, black, red, blue, purple, orange, green, grey
from collections import deque
from queue import PriorityQueue
import random
import math


def draw_searched_path(graph, edges, edge_id, start, goal, parent=None):
    """Draw searched path by traceback parent node from goal"""
    # Color start and goal node
    graph[start][3] = orange
    graph[goal][3] = purple

    # Set current node to goal
    current_node = goal

    if not parent:
        raise Exception("No parent list provided!")

    while True:
        # Color the edge between current node and it's parent
        edges[edge_id(current_node, parent[current_node])][1] = green
        # Move current node up
        current_node = parent[current_node]
        # Stop condition
        if current_node == start:
            break

    graphUI.updateUI()


def color(graph, edges, edge_id, start, goal, nodes=None, edge_path=None, edge_color=None):
    """
    @param nodes: array of dictionary which contained color node info
    Ex: nodes = [{"node": adjacent_node, "color": purple},
        {"node": start, "color": orange}]
    @param edge_path: array of edge id
    Ex: [1,2]
    """
    if nodes:
        for node in nodes:
            graph[node["node"]][3] = node["color"]
            graph[node["node"]][2] = white

    if edge_path and edge_color:
        edges[edge_id(edge_path[0], edge_path[1])][1] = edge_color

    if (edge_path and not edge_color) or (not edge_path and edge_color):
        raise Exception("Invalid input. Please provide enough arguments for edge!")

    graphUI.updateUI()


def calculate_euclide_dist(graph, start=None, goal=None):
    """Calculate Euclidean distance between two node"""
    if start == None and goal == None:
        raise Exception("Invalid input. Please provide enough arguments!")

    # cost = sqrt( (x-x')^2 + (y-y')^2 )
    return round(math.sqrt((graph[start][0][0] - graph[goal][0][0])**2 + (graph[start][0][-1] - graph[goal][0][-1])**2), 3)


def heuristic(graph, start, goal):
    """Heuristic function for a_star and greedy search"""
    if not start == None and goal == None:
        raise Exception("Invalid input. Please provide enough arguments!")

    return (graph[start][0][0] - graph[goal][0][0])**2 + (graph[start][0][-1] - graph[goal][0][-1])**2


def BFS(graph, edges, edge_id, start, goal):
    # Implement queue and parent dictionary for traceback
    queue = deque([start])
    parent = {start: None}

    while True:
        # Check if queue is empty
        if not queue:
            raise Exception("Can not find required path!")

        # Take out current node from the queue and color
        current_node = queue.popleft()
        color(graph, edges, edge_id, start, goal, nodes=[
              {"node": current_node, "color": yellow}])

        # Loop adjacent nodes and add to queue if it've not been discovered
        for adjacent_node in graph[current_node][1]:
            if adjacent_node not in parent:
                queue.append(adjacent_node)

                # Also add discovered node to parent dict
                parent[adjacent_node] = current_node
                # update color for discovered node and edge
                color(graph, edges, edge_id, start, goal, nodes=[
                    {"node": adjacent_node, "color": red}], edge_path=[current_node, adjacent_node], edge_color=white)

                # Check if adjacent node is goal node
                if adjacent_node == goal:
                    draw_searched_path(graph, edges, edge_id, start, goal, parent=parent)
                    return

        # Color current node
        color(graph, edges, edge_id, start, goal, nodes=[
            {"node": current_node, "color": blue}])


def DFS(graph, edges, edge_id, start, goal):
    # Implement queue
    queue = deque([start])
    # Implement discovered and parent dictionary for traceback
    parent = {start: None}

    while True:
        # Check if queue is empty
        if not queue:
            raise Exception("Can not find required path!")

        print(queue)

        # Take out current node from the queue and add it to discovered
        current_node = queue.popleft()
        # Color it
        color(graph, edges, edge_id, start, goal, nodes=[
            {"node": current_node, "color": yellow}])

        while True:
            # Check if all adjacent nodes have been discovered
            while set(graph[current_node][1]).issubset(set(parent)):
                # If all adjacent nodes of start have been discovered
                if current_node == start:
                    raise Exception("Can not find required path!")

                # Re-color it
                color(graph, edges, edge_id, start, goal, nodes=[
                    {"node": current_node, "color": blue}, {"node": parent[current_node], "color": yellow}])

                # Move current node backward
                current_node = parent[current_node]

            # Randomly pick an adjacent node
            discovered_node = random.choice(graph[current_node][1])

            if discovered_node not in parent:
                queue.appendleft(discovered_node)
                parent[discovered_node] = current_node
                # Update color for discovered node and edge
                color(graph, edges, edge_id, start, goal, nodes=[
                    {"node": discovered_node, "color": red}], edge_path=[current_node, discovered_node], edge_color=white)

                if discovered_node == goal:
                    # color start and goal node
                    draw_searched_path(graph, edges, edge_id, start, goal, parent=parent)
                    return

                break

        # Color current node
        color(graph, edges, edge_id, start, goal, nodes=[
            {"node": current_node, "color": blue}])


def UCS(graph, edges, edge_id, start, goal):
    # Implement queue
    queue = PriorityQueue()  # Use priority queue for auto sort
    queue.put((0, start))
    # Implement discovered and parent dictionary for traceback
    parent = {start: None}
    while True:
        # Check if queue is empty
        if queue.qsize() == 0:
            raise Exception("Can not find required path!")

        # Take out current node from the queue and color
        current = queue.get()

        current_node = current[1]
        current_cost = current[0]
        
        color(graph, edges, edge_id, start, goal, nodes=[
              {"node": current_node, "color": yellow}])

        # Loop adjacent nodes to and add to queue if it've not been discovered
        for adjacent_node in graph[current_node][1]:
            if adjacent_node not in parent:
                adjacent_cost = current_cost + calculate_euclide_dist(graph, current_node, adjacent_node)
                queue.put((adjacent_cost, adjacent_node))

                # Also add discovered node to parent dict
                parent[adjacent_node] = current_node
                # update color for discovered node and edge
                color(graph, edges, edge_id, start, goal, nodes=[
                    {"node": adjacent_node, "color": red}], edge_path=[current_node, adjacent_node], edge_color=white)

                # Check if adjacent node is goal node
                if adjacent_node == goal:
                    draw_searched_path(graph, edges, edge_id, start, goal, parent=parent)
                    return

        # Color current node
        color(graph, edges, edge_id, start, goal, nodes=[
            {"node": current_node, "color": blue}])

def AStar(graph, edges, edge_id, start, goal):
    # Implement queue
    fringe = PriorityQueue()  # Use priority queue for auto sort

    # Queue structure: (fScore, gScore, node_index)
    # Put 0 as start's fScore because we don't use it anyway, calculate it is a waste
    fringe.put((0, 0, start))

    # Implement discovered and parent dictionary for traceback
    parent = {start: None}

    while True:
        # Check if queue is empty
        if fringe.qsize() == 0:
            raise Exception("Can not find required path!")

        # Take out current node from the queue and color
        current = fringe.get()

        current_gScore = current[1]
        current_node = current[2]

        color(graph, edges, edge_id, start, goal, nodes=[
              {"node": current_node, "color": yellow}])

        # Loop adjacent nodes to and add to queue if it've not been discovered
        for adjacent_node in graph[current_node][1]:
            if adjacent_node not in parent:
                # f(n) = g(n) + h(n)
                adjacent_gScore = current_gScore + calculate_euclide_dist(graph, current_node, adjacent_node)
                adjacent_hScore = heuristic(graph, adjacent_node, goal)
                adjacent_fScore = adjacent_gScore + adjacent_hScore
                fringe.put((adjacent_fScore, adjacent_gScore, adjacent_node))

                # Also add discovered node to parent dict
                parent[adjacent_node] = current_node
                # update color for discovered node and edge
                color(graph, edges, edge_id, start, goal, nodes=[
                    {"node": adjacent_node, "color": red}], edge_path=[current_node, adjacent_node], edge_color=white)

                # Check if adjacent node is goal node
                if adjacent_node == goal:
                    draw_searched_path(graph, edges, edge_id, start, goal, parent=parent)
                    return

        # Color current node
        color(graph, edges, edge_id, start, goal, nodes=[
            {"node": current_node, "color": blue}])


def Greedy(graph, edges, edge_id, start, goal):
    # Implement queue
    fringe = PriorityQueue()  # Use priority queue for auto sort

    # Queue structure: (fScore, node_index)
    # Put 0 as start's fScore because we don't use it anyway, calculate it is a waste
    fringe.put((0, start))

    # Implement discovered and parent dictionary for traceback
    parent = {start: None}

    while True:
        # Check if queue is empty
        if fringe.qsize() == 0:
            raise Exception("Can not find required path!")

        # Take out current node from the queue and color
        current = fringe.get()

        current_fScore = current[0]
        current_node = current[1]

        color(graph, edges, edge_id, start, goal, nodes=[
              {"node": current_node, "color": yellow}])

        # Loop adjacent nodes to and add to queue if it've not been discovered
        for adjacent_node in graph[current_node][1]:
            if adjacent_node not in parent:
                # f(n) = h(n)
                adjacent_fScore = heuristic(graph, adjacent_node, goal)
                fringe.put((adjacent_fScore, adjacent_node))

                # Also add discovered node to parent dict
                parent[adjacent_node] = current_node
                # update color for discovered node and edge
                color(graph, edges, edge_id, start, goal, nodes=[
                    {"node": adjacent_node, "color": red}], edge_path=[current_node, adjacent_node], edge_color=white)

                # Check if adjacent node is goal node
                if adjacent_node == goal:
                    draw_searched_path(graph, edges, edge_id, start, goal, parent=parent)
                    return

        # Color current node
        color(graph, edges, edge_id, start, goal, nodes=[
            {"node": current_node, "color": blue}])


def example_func(graph, edges, edge_id, start, goal):
    """
    This function is just show some basic feature that you can use your project.
    @param graph: list - contain information of graph (same value as global_graph)
                    list of object:
                     [0] : (x,y) coordinate in UI
                     [1] : adjacent node indexes
                     [2] : node edge color
                     [3] : node fill color
                Ex: graph = [
                                [
                                    # position of node when draw on UI
                                    (139, 140),
                                    # list of adjacent node
                                    [1, 2],
                                    # grey - node edged color
                                    (100, 100, 100),
                                    # black - node fill color
                                    (0, 0, 0)
                                ],
                                [(312, 224), [0, 4, 2, 3],
                                  (100, 100, 100), (0, 0, 0)],
                                ...
                            ]
                It means this graph has Node 0 links to Node 1 and Node 2.
                Node 1 links to Node 0,2,3 and 4.
    @param edges: dict - dictionary of edge_id: [(n1,n2), color]. Ex: edges[edge_id(0,1)] = [(0,1), (0,0,0)] : set color
                    of edge from Node 0 to Node 1 is black.
    @param edge_id: id of each edge between two nodes. Ex: edge_id(0, 1) : id edge of two Node 0 and Node 1
    @param start: int - start vertices/node
    @param goal: int - vertices/node to search
    @return:
    """

    # Ex1: Set all edge from Node 1 to Adjacency node of Node 1 is green edges.
    node_1 = graph[1]
    for adjacency_node in node_1[1]:
        edges[edge_id(1, adjacency_node)][1] = green
    graphUI.updateUI()

    # Ex2: Set color of Node 2 is Red
    graph[2][3] = red
    graphUI.updateUI()

    # Ex3: Set all edge between node in a array.
    path = [4, 7, 9]  # -> set edge from 4-7, 7-9 is blue
    for i in range(len(path) - 1):
        edges[edge_id(path[i], path[i + 1])][1] = blue
    graphUI.updateUI()
