import collections

from numpy import Inf

from planner import Planner

'''
Below are example 1 and example 2 from my documentation.

Uncomment "p.Astar()" in each section to find solution.

the optimal path is saved to "path" folder

Author: Yu Hou
Last edit: 27/02/2020
'''

'''
Example 1:
'''
## whether treat wait cost as vertex cost
treat_wait_cost_as_vertex_cost = False

## create wait cost
wait_cost = collections.defaultdict(dict)
for i in range(1, 7):
    wait_cost[7 - i, 5][i, i] = Inf
for i in range(1, 9):
    wait_cost[5, 9 - i][i, i] = Inf

## create transition cost
transition_cost = collections.defaultdict(lambda: collections.defaultdict(dict))
for i in range(1, 7):
    transition_cost[7 - i, 5][i][8 - i, 5] = Inf
    transition_cost[8 - i, 5][i][7 - i, 5] = Inf
transition_cost[5, 5][2][5, 4] = Inf
transition_cost[5, 5][2][5, 6] = Inf
for i in range(1, 9):
    transition_cost[5, 9 - i][i][5, 10 - i] = Inf
    transition_cost[5, 10 - i][i][5, 9 - i] = Inf
transition_cost[5, 5][4][6, 5] = Inf
transition_cost[5, 5][4][4, 5] = Inf

## adds more arrow if wait cost is treated as edge cost.
if not treat_wait_cost_as_vertex_cost:
    for i in range(1, 7): transition_cost[7 - i, 5][i][6 - i, 5] = Inf
    for i in range(1, 9): transition_cost[5, 9 - i][i][5, 8 - i] = Inf

## plan the optimal path
start = (5, 4)
goal = (5, 9)
p = Planner('map/example1.map', wait_cost, transition_cost, start, goal, save_path=True,
            vertex_wait_cost=treat_wait_cost_as_vertex_cost)

## uncomment to find solution
# path = p.Astar()

'''
Example 2
wait cost is treated as edge cost
'''
## wait cost
wait_cost = collections.defaultdict(dict)
wait_cost[1, 1][1, 1] = 1000
wait_cost[1, 1][3, 4] = 200
wait_cost[1, 2][2, 3] = 1000
wait_cost[1, 2][4, 4] = 200
wait_cost[1, 2][5, 5] = 100

## transition cost
transition_cost = collections.defaultdict(lambda: collections.defaultdict(dict))
transition_cost[1, 1][3][1, 2] = 1000
transition_cost[1, 1][4][1, 2] = 200
transition_cost[1, 2][3][1, 1] = 100
transition_cost[1, 3][2][1, 2] = 400
transition_cost[1, 3][4][1, 2] = 200
transition_cost[1, 3][5][1, 2] = 1000

## plan the optimal path
start = (1, 1)
goal = (1, 3)
p = Planner('map/example2.map', wait_cost, transition_cost, start, goal, save_path=True, vertex_wait_cost=False)

## uncomment to find solution
# p.Astar()
