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

    def updateFrontiers(self):
        pass

    def chooseNewDestination(self):


#         print 'blackList:'
#         for coords in self.blackList:
#             print str(coords)

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
		minVal = 2000000.0
		nextCandidate = None

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
            
