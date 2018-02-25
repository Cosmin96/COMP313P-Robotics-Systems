#! /usr/bin/env python

# See run_fifo_standalone.py for documentation. The only difference is that
# a LIFO planner is created instead of a FIFO planner.

from comp313p_planner_controller.occupancy_grid import OccupancyGrid
from comp313p_planner_controller.astar_planner import AStarPlanner

occupancyGrid = OccupancyGrid(21, 21, 0.5)

for y in xrange(0, 20):
    occupancyGrid.setCell(11, y, 1)

start = (3, 18)
goal = (20, 0)
heuristics = ["zero", "constant", "euclidean", "octile", "manhattan", "chebyshev", "minkowski", "cosine", "canberra"]

for heuristic in heuristics:
    planner = AStarPlanner('A* Search - ' + heuristic, occupancyGrid, heuristic, scale = 10)
    planner.setRunInteractively(True)

    planner.setWindowHeightInPixels(400)

    goalReached = planner.search(start, goal)

    path = planner.extractPathToGoal()
