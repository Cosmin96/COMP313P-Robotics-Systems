#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose
from comp313p_planner_controller.planned_path import PlannedPath
from comp313p_planner_controller.controller_base import ControllerBase
import math
import angles

# This sample controller extends the controller in lab2. It keeps going towards next waypoint
# by rotating and then moving forward. Also, it includes a proportional controller to counter
# the error encountered along the way (the larger the error, the higher the counter move)
class Move2GoalController(ControllerBase):

    def __init__(self, occupancyGrid):
        ControllerBase.__init__(self, occupancyGrid)
        # Get the proportional gain settings
        self.distanceErrorGain = rospy.get_param('distance_error_gain', 1)
        self.angleErrorGain = rospy.get_param('angle_error_gain', 4)
        self.driveAngleErrorTolerance = math.radians(rospy.get_param('angle_error_tolerance', 1))

        
    # Driving to a given waypoint from current position
    def driveToWaypoint(self, waypoint):
        vel_msg = Twist()
        # Compute the needed distance and angle errors
        dist_err = math.sqrt((waypoint[0] - self.pose.x) ** 2 + (waypoint[1] - self.pose.y) ** 2)
        angle_err = self.smallestAngDist(self.pose.theta, math.atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x))
       
        # Keep until in the error margin
        while (dist_err >= self.distanceErrorTolerance) and (not rospy.is_shutdown()):
            # Proportional Controller for linear velocity
            if math.fabs(angle_err) < self.driveAngleErrorTolerance:
                vel_msg.linear.x = max(0.0, min(self.distanceErrorGain * dist_err * 5, 50.0))
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

            # Proportional Controller for angular velocity
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(self.angleErrorGain * angle_err, 5.0))

            # Publishing message
            self.velocityPublisher.publish(vel_msg)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()
                
            self.rate.sleep()

            dist_err = math.sqrt((waypoint[0] - self.pose.x) ** 2 + (waypoint[1] - self.pose.y) ** 2)
            angle_err = self.smallestAngDist(self.pose.theta, math.atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x))

        # Stop moving
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0

        # Publishing message
        self.velocityPublisher.publish(vel_msg)


    # Rotating to a goal angle
    def rotateToGoalOrientation(self, goal_orient):
        vel_msg = Twist()
        goal_orient = math.radians(goal_orient)
        angle_err = self.smallestAngDist(self.pose.theta, goal_orient)

        # Keep until in the error margin
        while (math.fabs(angle_err) >= self.goalAngleErrorTolerance) and (not rospy.is_shutdown()):
            # Proportional Controller for angular velocity
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(self.angleErrorGain * angle_err, 5.0))

            # Publishing message
            self.velocityPublisher.publish(vel_msg)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()
                
            self.rate.sleep()

            # Update angle error
            angle_err = self.smallestAngDist(self.pose.theta, goal_orient)

        # Stop moving
        vel_msg.angular.z = 0

        # Publishing message
        self.velocityPublisher.publish(vel_msg)


    # Returns the smallest angle difference
    def smallestAngDist(self, fromA, toB):
        diff = toB - fromA

        # Check if we should add/substract 2PI based on difference
        if (diff < (-math.pi)):
            diff = diff + 2.0*math.pi
        elif(diff > math.pi):
            diff = diff - 2.0*math.pi
        return diff
        #!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose
from comp313p_planner_controller.planned_path import PlannedPath
from comp313p_planner_controller.controller_base import ControllerBase
import math
import angles

# This sample controller extends the controller in lab2. It keeps going towards next waypoint
# by rotating and then moving forward. Also, it includes a proportional controller to counter
# the error encountered along the way (the larger the error, the higher the counter move)
class Move2GoalController(ControllerBase):

    def __init__(self, occupancyGrid):
        ControllerBase.__init__(self, occupancyGrid)
        # Get the proportional gain settings
        self.distanceErrorGain = rospy.get_param('distance_error_gain', 1)
        self.angleErrorGain = rospy.get_param('angle_error_gain', 4)
        self.driveAngleErrorTolerance = math.radians(rospy.get_param('angle_error_tolerance', 1))

        
    # Driving to a given waypoint from current position
    def driveToWaypoint(self, waypoint):
        vel_msg = Twist()
        # Compute the needed distance and angle errors
        dist_err = math.sqrt((waypoint[0] - self.pose.x) ** 2 + (waypoint[1] - self.pose.y) ** 2)
        angle_err = self.smallestAngDist(self.pose.theta, math.atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x))
       
        # Keep until in the error margin
        while (dist_err >= self.distanceErrorTolerance) and (not rospy.is_shutdown()):
            # Proportional Controller for linear velocity
            if math.fabs(angle_err) < self.driveAngleErrorTolerance:
                vel_msg.linear.x = max(0.0, min(self.distanceErrorGain * dist_err * 5, 50.0))
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

            # Proportional Controller for angular velocity
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(self.angleErrorGain * angle_err, 5.0))

            # Publishing message
            self.velocityPublisher.publish(vel_msg)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()
                
            self.rate.sleep()

            dist_err = math.sqrt((waypoint[0] - self.pose.x) ** 2 + (waypoint[1] - self.pose.y) ** 2)
            angle_err = self.smallestAngDist(self.pose.theta, math.atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x))

        # Stop moving
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0

        # Publishing message
        self.velocityPublisher.publish(vel_msg)


    # Rotating to a goal angle
    def rotateToGoalOrientation(self, goal_orient):
        vel_msg = Twist()
        goal_orient = math.radians(goal_orient)
        angle_err = self.smallestAngDist(self.pose.theta, goal_orient)

        # Keep until in the error margin
        while (math.fabs(angle_err) >= self.goalAngleErrorTolerance) and (not rospy.is_shutdown()):
            # Proportional Controller for angular velocity
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(self.angleErrorGain * angle_err, 5.0))

            # Publishing message
            self.velocityPublisher.publish(vel_msg)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()
                
            self.rate.sleep()

            # Update angle error
            angle_err = self.smallestAngDist(self.pose.theta, goal_orient)

        # Stop moving
        vel_msg.angular.z = 0

        # Publishing message
        self.velocityPublisher.publish(vel_msg)


    # Returns the smallest angle difference
    def smallestAngDist(self, fromA, toB):
        diff = toB - fromA

        # Check if we should add/substract 2PI based on difference
        if (diff < (-math.pi)):
            diff = diff + 2.0*math.pi
        elif(diff > math.pi):
            diff = diff - 2.0*math.pi
        return diff
        
