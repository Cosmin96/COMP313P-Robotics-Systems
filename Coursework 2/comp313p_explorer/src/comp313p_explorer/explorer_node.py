import rospy
import math

from explorer_node_base import ExplorerNodeBase

# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)

	self.blackList = []
    	self.currPos = None


    def getCandidateInSegmentation(self):
        currCandidate = self.currPos

        if(self.currPos is None):
            return None
        
        row_sz = self.occupancyGrid.getWidthInCells() / 6
        col_sz = self.occupancyGrid.getHeightInCells() / 4

        for i in range(0, row_sz):
            for j in range(0, col_sz):
                if((i * row_sz) <= currCandidate[0] and ((i + 1) * row_sz) > currCandidate[0] and (j * col_sz) <= currCandidate[1] and ((j + 1) * col_sz) >= currCandidate[1]):
                    return ( i * row_sz, (i + 1) * row_sz,  j * col_sz, (j + 1) * col_sz )

        return None

                        


    # returns false if nor more frontiers
    def updateFrontiers(self):        
	for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                candidate = (x, y)
                if self.isFrontierCell(x, y) is True:
                    candidateGood = True

		    if hasattr(self, 'blackList'):
                    	for k in range(0, len(self.blackList)):
                        	if self.blackList[k] == candidate:
                            		candidateGood = False
                            		break

                    if candidateGood is True:
                        return True
                        
        return False

    def chooseNewDestination(self):
        if(self.currPos is None):
            for x in range(0, self.occupancyGrid.getWidthInCells()):
                for y in range(0, self.occupancyGrid.getHeightInCells()):
                    candidate = (x, y)
                    if self.isFrontierCell(x, y) is True:
                        candidateGood = True
                        for k in range(0, len(self.blackList)):
                            if self.blackList[k] == candidate:
                                candidateGood = False
                                break

                        if candidateGood is True:
                            self.currPos = candidate
                            return True, candidate
            
            self.currPos = None
            return False, None

        else:
            coords = self.getCandidateInSegmentation()
            minVal = 2000000.0
            nextCandidate = None

	    if coords is not None:
		    for x in range(coords[0], coords[1]):
		        for y in range(coords[2], coords[3]):
		            candidate = (x, y)
		            if self.isFrontierCell(x, y) is True:
		                candidateGood = True
		                for k in range(0, len(self.blackList)):
		                    if self.blackList[k] == candidate:
		                        candidateGood = False
		                        break

		                if candidateGood is True:
		                    distanceToCandidate = math.sqrt((candidate[0] - self.currPos[0]) ** 2 + (candidate[1] - self.currPos[1]) ** 2)
		                    if distanceToCandidate < minVal:
		                       minVal = distanceToCandidate
		                       nextCandidate = candidate

		    if(nextCandidate is not None):
		        self.currPos = nextCandidate
		        return True, nextCandidate

            for x in range(0, self.occupancyGrid.getWidthInCells()):
                for y in range(0, self.occupancyGrid.getHeightInCells()):
                    candidate = (x, y)
                    if self.isFrontierCell(x, y) is True:
                        candidateGood = True
                        for k in range(0, len(self.blackList)):
                            if self.blackList[k] == candidate:
                                candidateGood = False
                                break

                        if candidateGood is True:
                            distanceToCandidate = math.sqrt((candidate[0] - self.currPos[0]) ** 2 + (candidate[1] - self.currPos[1]) ** 2)
                            if distanceToCandidate < minVal:
                               minVal = distanceToCandidate
                               nextCandidate = candidate
            
            self.currPos = nextCandidate        

            if(nextCandidate is not None):
                return True, nextCandidate
            else:
                return False, None
            

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
            

            
