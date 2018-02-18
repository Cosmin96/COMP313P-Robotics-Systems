# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from heapq import heappush, heappop


class DijkstraPlanner(CellBasedForwardSearch):
    # This implements a simple LIFO search algorithm

    def __init__(self, title, occupancyGrid, start = (0, 0)):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.lifoQueue = list()
        self.dijkstra(self.occupancyGrid, start)


    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        heappush(self.lifoQueue, (cell.pathCost, cell))

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

    def dijkstra(self, occupancyGrid, source):
        start = occupancyGrid.getCell(source[0], source[1])
        start.pathCost = 0
        Q = [(0, start)]


        while len(Q) != 0:
            (cost, cell) = heappop(Q)

            for cell2 in self.getNextSetOfCellsToBeVisited(cell):
                if cell2.pathCost > cell.pathCost + 1:
                    cell2.pathCost = cell.pathCost + 1
                    heappush(Q, (cell2.pathCost, cell2))
