import numpy as np
import cv2
import tcod

cost = [[1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1], [
    1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1]]

# maze = [[0, 1, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [
#         0, 1, 0, 0, 0, 0, 0, 1, 0, 0], [0, 0, 1, 0, 0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

aStar = tcod.path.AStar(cost, 0.0)
temp = aStar.get_path(5, 0, 0, 5)
print("AStar: ", temp)

cost = np.ones((6, 6), dtype=np.int8, order="C")
graph = tcod.path.SimpleGraph(cost=cost, cardinal=2, diagonal=3)
pf = tcod.path.Pathfinder(graph)
pf.add_root((5, 0))
a = pf.path_to((0, 5)).tolist()
print("Pathfinder: ", a)

commands_list = a

commands_meters = commands_list
for i in range(len(commands_list)):
    for j in range(2):
        if commands_list[i][j] == 0:
            commands_meters[i][j] = -10
        elif commands_list[i][j] > 0:
            if commands_list[i][j] < 256:
                commands_meters[i][j] = -10 + (commands_list[i][j]*10/512)
            elif commands_list[i][j] == 256:
                commands_meters[i][j] = 0
            elif commands_list[i][j] == 512:
                commands_meters[i][j] = 10
            else:
                commands_meters[i][j] = commands_list[i][j]*10/512

print(commands_meters)


# path = a
# for i in range(len(path) - 1):
#     if path[i][0] == path[i+1][0]:
#         if path[i][1] > path[i+1][1]:
#             print("Izquierda")
#         else:
#             print("Derecha")
#     elif path[i][1] == path[i+1][1]:
#         # print("Hello")
#         if path[i][0] > path[i+1][0]:
#             print("Norte")
#         else:
#             print("Sur")
#     elif path[i][0] > path[i+1][0]:
#         if path[i][1] > path[i+1][1]:
#             print("Diagonal superior izquierda")
#         elif path[i][1] < path[i+1][1]:
#             print("Diagonal superior derecha")
#     elif path[i][0] < path[i+1][0]:
#         if path[i][1] > path[i+1][1]:
#             print("Diagonal inferior izquierda")
#         elif path[i][1] < path[i+1][1]:
#             print("Diagonal inferior derecha")
