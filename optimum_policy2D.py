# ----------
# User Instructions:
#
# Implement the function optimum_policy2D below.
#
# You are given a car in grid with initial state
# init. Your task is to compute and return the car's
# optimal path to the position specified in goal;
# the costs for each motion are as defined in cost.
#
# There are four motion directions: up, left, down, and right.
# Increasing the index in this array corresponds to making a
# a left turn, and decreasing the index corresponds to making a
# right turn.

forward = [
    [-1, 0],  # go up
    [0, -1],  # go left
    [1, 0],  # go down
    [0, 1]
]  # go right
forward_name = ['^', '<', 'v', '>']
# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

# EXAMPLE INPUTS:
# grid format:
#     0 = navigable space
#     1 = unnavigable space
grid = [[1, 1, 1, 0, 0, 0], [1, 1, 1, 0, 1, 0], [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1], [1, 1, 1, 0, 1, 1]]
init = [4, 3, 0]  # given in the form [row,col,direction]
# direction = 0: up
#             1: left
#             2: down
#             3: right

goal = [2, 0]  # given in the form [row,col]
cost = [2, 1, 20]  # cost has 3 values, corresponding to making


# a right turn, no turn, and a left turn

# EXAMPLE OUTPUT:
# calling optimum_policy2D with the given parameters should return
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]
# ----------

# ----------------------------------------
# modify code below
# ----------------------------------------


def optimum_policy2D(grid, init, goal, cost):
    value = [[[999 for _ in range(len(grid[0]))] for _ in range(len(grid))]
             for _ in range(4)]
    policy = [[[' ' for _ in range(len(grid[0]))] for _ in range(len(grid))]
              for _ in range(4)]
    policy2D = [[' ' for _ in range(len(grid[0]))] for _ in range(len(grid))]
    plan = [[[' ' for _ in range(len(grid[0]))] for _ in range(len(grid))]
            for _ in range(4)]
    change = True
    while change:
        change = False
        for x in range(len(grid)):
            for y in range(len(grid[0])):
                for d in range(4):
                    if x == goal[0] and y == goal[1]:
                        if value[d][x][y] > 0:
                            change = True
                            value[d][x][y] = 0
                            policy[d][x][y] = '*'
                            plan[d][x][y] = '*'
                    elif grid[x][y] == 0:
                        for i in range(len(action)):
                            d2 = (d + action[i]) % 4
                            x2 = x + forward[d2][0]
                            y2 = y + forward[d2][1]
                            if 0 <= x2 < len(grid) and 0 <= y2 < len(
                                    grid[0]) and grid[x2][y2] == 0:
                                if value[d][x][y] > value[d2][x2][y2] + cost[i]:
                                    policy[d][x][y] = action_name[i]
                                    plan[d][x][y] = forward_name[i]
                                    change = True
                                    value[d][x][
                                        y] = value[d2][x2][y2] + cost[i]
    x, y, d = init
    policy2D[x][y] = policy[d][x][y]
    while policy[d][x][y] is not '*':
        d2 = (d + action[action_name.index(policy[d][x][y])]) % 4
        x += forward[d2][0]
        y += forward[d2][1]
        d = d2
        policy2D[x][y] = policy[d][x][y]
    return policy2D, policy[init[-1]]


def main():
    policy2D, policy = optimum_policy2D(grid, init, goal, cost)
    for _ in policy2D:
        print _
    print ''
    for _ in policy:
        print _


if __name__ == '__main__':
    main()
