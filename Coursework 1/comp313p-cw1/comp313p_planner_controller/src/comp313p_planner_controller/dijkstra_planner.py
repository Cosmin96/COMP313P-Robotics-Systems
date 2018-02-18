# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from heapq import heappush, heappop
import math

class DijkstraPlanner(CellBasedForwardSearch):
    # This implements a simple LIFO search algorithm

    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.lifoQueue = list()

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        if cell.parent is None:
            cell.pathCost = 0
        else:
            distance = self.getDistance(cell, cell.parent)
            cell.pathCost = cell.parent.pathCost + distance

        heappush(self.lifoQueue, (cell.pathCost, cell))

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.lifoQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = heappop(self.lifoQueue)[1]
        return cell

    def resolveDuplicate(self, cell, parentCell):
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