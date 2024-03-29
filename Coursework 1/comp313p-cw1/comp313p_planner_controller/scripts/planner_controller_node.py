#!/usr/bin/env python
import rospy
import threading
import math
import time
import datetime

# Robot pose
from geometry_msgs.msg import Pose

# Response we get from the map server when we ask for the map
from nav_msgs.srv import GetMap

# The service messages this node sends. This is actually a report
# that the robot has reached its goal
from comp313p_planner_controller.srv import *

# The occupancy grid, used to store our representation of the map
from comp313p_planner_controller.occupancy_grid import OccupancyGrid

# The planner used to figure out the path
from comp313p_planner_controller.fifo_planner import FIFOPlanner
from comp313p_planner_controller.lifo_planner import LIFOPlanner
from comp313p_planner_controller.astar_planner import AStarPlanner
from comp313p_planner_controller.bestFirst_planner import BestFirstPlanner
from comp313p_planner_controller.dijkstra_planner import DijkstraPlanner

# The controller to drive the robot along the path
from comp313p_planner_controller.move2goal_controller import Move2GoalController

# Self class interfaces with the planner and the controller
class PlannerControllerNode(object):

    def __init__(self):
        rospy.init_node('planner_controller', anonymous=True)
        
        self.waitForGoal =  threading.Condition()
        self.waitForDriveCompleted =  threading.Condition()
        self.goal = None
        pass
    
    def createOccupancyGridFromMapServer(self):
        # Get the map service
        rospy.loginfo('Waiting for static_map to become available.')
        rospy.wait_for_service('static_map') 
        self.mapServer = rospy.ServiceProxy('static_map', GetMap)
        rospy.loginfo('Found static_map; requesting map data')

        # Query the map status
        response = self.mapServer()
        map = response.map
        rospy.loginfo('Got map data')

        # Allocate the occupancy grid and set the data from the array sent back by the map server
        self.occupancyGrid = OccupancyGrid(map.info.width, map.info.height, map.info.resolution)
        self.occupancyGrid.setScale(rospy.get_param('plan_scale', 5))
        self.occupancyGrid.setFromDataArrayFromMapServer(map.data)
        self.occupancyGrid.expandObstaclesToAccountForCircularRobotOfRadius(0.2)

    def createPlanner(self):
	heuristic = 'octile'
        self.planner = AStarPlanner('A* Search - ' + heuristic, self.occupancyGrid, heuristic, scale = 10)
	#self.planner = BestFirstPlanner('Best First Search', self.occupancyGrid)
	#self.planner = DijkstraPlanner('Dijkstra', self.occupancyGrid)
	#self.planner = FIFOPlanner('FIFO', self.occupancyGrid)
	#self.planner = LIFOPlanner('LIFO', self.occupancyGrid)
        self.planner.setPauseTime(0)
        self.planner.windowHeightInPixels = rospy.get_param('maximum_window_height_in_pixels', 700)
        
    def createRobotController(self):
        self.robotController = Move2GoalController(self.occupancyGrid)

    def handleDriveToGoal(self, goal):
        # Report to the main loop that we have a new goal
        self.waitForGoal.acquire()
        self.goal = goal
        self.waitForGoal.notify()
        self.waitForGoal.release()

        # Wait until the robot has finished driving
        self.waitForDriveCompleted.acquire()
        self.waitForDriveCompleted.wait()
        self.waitForDriveCompleted.release()

        return GoalResponse(True)

    # Run the planner. Note that we do not take account of the robot orientation when planning.
    # The reason is simplicity; adding orientation means we have a full 3D planning problem.
    # As a result, the plan will not be as efficient as it could be.
    def driveToGoal(self, goal):

        # Get the current pose of the robot
        pose = self.robotController.getCurrentPose()
        start = (pose.x, pose.y)

        print "start = " + str(start)
        print "goal = " + str(goal)
        
        # Call the planner
        startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        print "startCellCoords = " + str(startCellCoords)
        print "goalCellCoords = " + str(goalCellCoords)

         # Exit if we need to
        if rospy.is_shutdown() is True:
            return False

        # Get the plan
        goalReached = self.planner.search(startCellCoords, goalCellCoords)

        # Exit if we need to
        if rospy.is_shutdown() is True:
            return False

        # If we can't reach the goal, give up and return
        if goalReached is False:
            rospy.logwarn("Could not reach the goal at (%d, %d); moving to next goal", \
                          goalCellCoords[0], goalCellCoords[1])
            return False
        
        # Extract the path
        path = self.planner.extractPathToGoal()

        # Now drive it
        self.robotController.drivePathToGoal(path, goal.theta, self.planner.getPlannerDrawer())

        return True
    
    def run(self):

        # First set up the occupancy grid
        self.createOccupancyGridFromMapServer()

        # Create the planner
        self.createPlanner()
        
        # Set up the robot controller
        self.createRobotController()

        # Set up the wait for the service. Note that we can't directly
        # handle all the driving operations in the service
        # handler. The reason is that the planner can create a GUI,
        # and self MUST run in the main thread. The result is pretty
        # ugly logic and can lead to deadlocking.
        service = rospy.Service('drive_to_goal', Goal, self.handleDriveToGoal)

        print 'Spinning to service goal requests'
        
        while not rospy.is_shutdown():

            # Wait for a new goal. Allow at most 0.1s, which gives
            # time to check if we are shutting down
            self.waitForGoal.acquire()
            self.waitForGoal.wait(0.1)
            self.waitForGoal.release()

            # If no goal has been allocated, cycle around
            if (self.goal is None):
                continue

            self.driveToGoal(self.goal)
            print("---------")
            print("---------", "Total distance traveled:", self.robotController.total_dist)
            print("---------", "Total angle turned:", self.robotController.total_angle)
            print("---------", "Total time spent:", self.robotController.total_time)
            print("---------")
            self.robotController.total_dist = 0.0
            self.robotController.total_angle = 0.0
            self.robotController.total_time = 0.0
            self.goal = None

            # Signal back to the service handler that we are done
            self.waitForDriveCompleted.acquire()
            self.waitForDriveCompleted.notify()
            self.waitForDriveCompleted.release()

if __name__ == '__main__':
    try:
        plannerController = PlannerControllerNode()
        plannerController.run()
    except rospy.ROSInterruptException:
        pass
