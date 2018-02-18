# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from cell import CellLabel
from heapq import heappush, heappop
import math

class AStarPlanner(CellBasedForwardSearch):
    # This implements a simple LIFO search algorithm

    def __init__(self, title, occupancyGrid, heuristic, scale = 0):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.lifoQueue = list()
        self.heuristic = heuristic
        self.scale = scale


    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        if cell.parent is not None:
            distance = self.getDistance(cell, cell.parent)
            cell.pathCost = cell.parent.pathCost + distance
        heappush(self.lifoQueue, (cell.pathCost + self.applyHeuristic(cell) ,cell))

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.lifoQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = heappop(self.lifoQueue)[1]
        return cell

    def resolveDuplicate(self, cell, parentCell):
        if cell.label != CellLabel.DEAD:
            distance = self.getDistance(cell, parentCell)
            newCost = parentCell.pathCost + distance
            if newCost < cell.pathCost:
                cell.pathCost = newCost
                cell.parent = parentCell


    def getDistance(self, cell, parentCell):
        cellX = cell.coords[0]
        cellY = cell.coords[1]
        parentX = parentCell.coords[0]
        parentY = parentCell.coords[1]
        return math.sqrt(((cellX - parentX) ** 2) + ((cellY - parentY) ** 2))

    def applyHeuristic(self, cell):

        h = 0

        if self.heuristic is "zero":
            h = 0

        if self.heuristic is "constant":
            h = ord('A') + ord('F')

        if self.heuristic is "euclidean":
            h = self.getDistance(cell, self.goal)

        if self.heuristic is "octile":
            x = abs(cell.coords[0] - self.goal.coords[0])
            y = abs(cell.coords[1] - self.goal.coords[1])
            h = max(x, y) + (math.sqrt(2) - 1) * min(x, y)

        if self.heuristic is "manhattan":
            x = abs(cell.coords[0] - self.goal.coords[0])
            y = abs(cell.coords[1] - self.goal.coords[1])
            h = x + y

        return h * (self.scale + 1)
