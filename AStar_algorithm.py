import numpy as np
import cv2


class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.start_distance = 0
        self.end_distance = 0
        self.total_score = 0

    def __eq__(self, other):
        return self.position == other.position


def return_path(current_node, maze):
    path = []
    detailed_commands = []
    general_commands = []
    num_rows, num_cols = np.shape(maze)
    result = [[-1 for i in range(num_cols)] for j in range(num_rows)]
    current = current_node

    while current is not None:
        path.append(current.position)
        # print(current.position)
        current = current.parent
    path = path[::-1]  # Return reversed path

    for i in range(len(path) - 1):
        command = [path[i+1][0], path[i+1][1]]
        if (path[i+1][0] == path[i][0]):
            if (path[i+1][1] > path[i][1]):
                # print("Move East towards position ", path[i+1])
                # print("[", path[i+1][0], ",", path[i+1][1], ", 90 ]")
                command.append(90)
            else:
                # print("Move West towards position ", path[i+1])
                # print("[", path[i+1][0], ",", path[i+1][1], ", 270 ]")
                command.append(270)
        elif (path[i+1][0] < path[i][0]):
            # print("Move North towards position ", path[i+1])
            # print("[", path[i+1][0], ",", path[i+1][1], ", 180 ]")
            command.append(180)
        else:
            # print("Move South towards position ", path[i+1])
            # print("[", path[i+1][0], ",", path[i+1][1], ", 0 ]")
            command.append(0)
        detailed_commands.append(command)

    print("Detailed commands:")
    print(detailed_commands)

    # Get list of final commands for ground robot

    # Posiblemente haya que agregar como primer entrada el punto de partida
    # con la orientación que tenga el robot. REVISAR.
    general_commands.append(detailed_commands[0])

    for i in range(len(detailed_commands)-1):
        if (detailed_commands[i+1][2] != detailed_commands[i][2]):
            general_commands.append(detailed_commands[i])

    # Add the END point to command list
    general_commands.append(detailed_commands[-1])

    print("General commands")
    print(general_commands)

    # Get the path in a matrix form
    start_value = 0
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value += 1
    return result


def ASearch(maze, cost, start, end):
    start_node = Node(None, tuple(start))
    start_node.start_distance = start_node.end_distance = start_node.total_score = 0
    end_node = Node(None, tuple(end))
    end_node.start_distance = end_node.end_distance = end_node.total_score = 0

    # Initialize both visited and yet_to_visit list
    yet_to_visit_list = []
    visited_list = []
    yet_to_visit_list.append(start_node)

    outer_iterations = 0
    max_iterations = (len(np.floor_divide(maze, 2))) ** 10

    move = [[-1, 0], [0, -1], [1, 0], [0, 1]]

    num_rows, num_cols = np.shape(maze)

    while len(yet_to_visit_list) > 0:
        outer_iterations += 1
        current_node = yet_to_visit_list[0]
        current_index = 0
        for index, item in enumerate(yet_to_visit_list):
            if item.total_score < current_node.total_score:
                current_node = item
                current_index = index

        if outer_iterations > max_iterations:
            print("Giving up on pathfinding too many iterations")
            return return_path(current_node, maze)

        yet_to_visit_list.pop(current_index)
        visited_list.append(current_node)

        if current_node == end_node:
            return return_path(current_node, maze)

        children = []

        for new_position in move:  # Adjacent movement options

            # Get node position
            node_position = (
                current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range (maze boundaries)
            if node_position[0] > (num_rows - 1) or node_position[0] < 0 or node_position[1] > (num_cols - 1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain (no obstacles)
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node and add it to the list
            new_node = Node(current_node, node_position)
            children.append(new_node)

        # Loop through children
        for child in children:
            # Child is on the visited list
            if len([visited_child for visited_child in visited_list if visited_child == child]) > 0:
                continue

            # Create the total_score, start_distance, and end_distance values
            child.start_distance = current_node.start_distance + cost
            child.end_distance = ((child.position[0] - end_node.position[0]) ** 2) + (
                (child.position[1] - end_node.position[1]) ** 2)
            child.total_score = child.start_distance + child.end_distance

            # Child is already in the yet_to_visit list
            if len([i for i in yet_to_visit_list if child == i and child.start_distance > i.start_distance]) > 0:
                continue

            # Add the child to the yet_to_visit list
            yet_to_visit_list.append(child)


def createMap(grid_matrix):
    map_matrix = np.copy(grid_matrix)

    size_y = len(grid_matrix)
    size_x = len(grid_matrix[0])

    for y in range(size_y):
        for x in range(size_x):
            if (grid_matrix[y][x] == 1):
                if (x > 0):
                    map_matrix[y][x-1] = 1
                if (x < size_x-1):
                    map_matrix[y][x+1] = 1
                if (y > 0):
                    map_matrix[y-1][x] = 1
                if (y < size_y-1):
                    map_matrix[y+1][x] = 1
    return map_matrix


if __name__ == '__main__':
    maze = [[0, 1, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [
        0, 1, 0, 0, 0, 0, 0, 1, 0, 0], [0, 0, 1, 0, 0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
    start = [5, 0]
    end = [0, 9]
    cost = 1
    maze_final = createMap(maze)
    path = ASearch(maze_final, cost, start, end)

    # Print matrix path
    print('\n'.join([''.join(["{:" ">3d}".format(item)
                              for item in row]) for row in path]))
