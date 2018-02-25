# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from heapq import heappush, heappop
import math

class BestFirstPlanner(CellBasedForwardSearch):
    # This implements a simple LIFO search algorithm

    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.lifoQueue = list()

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
	goalX = self.goal.coords[0]
	goalY = self.goal.coords[1]
	cellX = cell.coords[0]
	cellY = cell.coords[1]
	distance = math.sqrt(((goalX - cellX) ** 2) + ((goalY - cellY) ** 2))
        heappush(self.lifoQueue, (distance, cell))

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.lifoQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = heappop(self.lifoQueue)[1]
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass
