# coding: utf-8

# In[228]:

# ----------
# User Instructions:
#
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0], [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid) - 1, len(grid[0]) - 1]
cost = 1

delta = [
    [-1, 0],  # go up
    [0, -1],  # go left
    [1, 0],  # go down
    [0, 1]
]  # go right

delta_name = ['^', '<', 'v', '>']


def search(grid, init, goal, cost):
    # ----------------------------------------
    # insert code here
    # ----------------------------------------
    arrCheck = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    arrCheck[init[0]][init[1]] = 1
    g = 0
    x = init[0]
    y = init[1]
    listOpen = [[g, x, y]]

    found = False
    resign = False
    count = 0
    while not found and not resign:
        count += 1
        if len(listOpen) == 0:
            resign = True
            return 'fail'
        else:
            listOpen.sort(key=lambda x: x[0], reverse=True)
            print listOpen
            g, x, y = listOpen.pop()

            if x == goal[0] and y == goal[1]:
                found = True
                print 'Tried', count, 'times'
                return [g, x, y]
            else:
                for actionID, action in enumerate(delta):
                    xNew = x + action[0]
                    yNew = y + action[1]
                    if 0 <= xNew <= len(grid) - 1 and 0 <= yNew <= len(
                            grid[0]) - 1:
                        a = [g, x,
                             y], delta_name[actionID], [g + cost, xNew, yNew]
                        if grid[xNew][yNew] == 0 and arrCheck[xNew][yNew] != 1:
                            arrCheck[xNew][yNew] = 1
                            #                             print a, 'open'
                            listOpen.append([g + cost, xNew, yNew])


print search(grid, init, goal, cost)
