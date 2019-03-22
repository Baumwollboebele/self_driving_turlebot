import rospy
import time
import math

from geometry_msgs.msg import Twist


# Class that holds all the Constants
class NavConstants(object):

#Constants for driving speed
    STOP = 0
    SLOW_VEL = 0.1
    NORMAL_VEL = 0.5
    FAST_VEL = 0.7
    WARP_5 = 831600000000

#Constants for turning
    DONT_TURN = 0
    SLOW_TURN_RIGHT = -0.1
    NORMAL_TURN_RIGHT = -0.2
    FAST_TURN_RIGHT = -0.4

    SLOW_TURN_LEFT = 0.1
    NORMAL_TURN_LEFT = 0.2
    FAST_TURN_LEFT = 0.4

#Constants for Distance
    COLISSION_RANGE = 1
    NEAR_GOAL = 1
    AT_GOAL = 0.2
    OBSTACLE_DISTANCE = 1

#Constants for laser index
    FOLLOW_WALL_MAX = 95
    FOLLOW_WALL_MIN = 85

#Constants for precision
    ANGLE_THRESHOLD_MIN = -1
    ANGLE_THRESHOLD_MAX = 1


#The class implements functions which are responsible for the robots movement
class Navigator(object):

    r = None
    orders = Twist()
    velocity = 0
    rotation = 0

    pub = None
    rate = None


# Gets the robot object and initialises it as a class variable,
# also initiates the publisher which is responsible for publishing the rotation and velocity of the robot.

    def __init__(self,object):
        self.r = object
        self.pub = rospy.Publisher('/cmd_vel', Twist)




#This Function is the main logic which is responsible for trajectory of the robot.
#It consists of three states: Goal reached, Obstacle detected and No Obstacle detected.
    def drive(self):
        while True:
            if self.r.valuesInitiated() is True:
                #If the robot is closer to a goal then the AT_GOAL variable it stops for 5 seconds and the next goal is set.
                if self.r.goalDistance() < NavConstants.AT_GOAL:
                    self.velocity = NavConstants.STOP
                    self.rotation = NavConstants.DONT_TURN

                    print "Goal " + str(self.r.getCurrentGoal()+1) + " reached !"

                    self.orders.linear.x = self.velocity
                    self.orders.angular.z = self.rotation
                    self.pub.publish(self.orders)

                    time.sleep(5)

                    self.r.incCurrentGoal(1)

                #If the way to the goal is obstructed, the robot then turns in a right angle to the obstacle it detected.
                elif self.isObstacle() is True:
                    if self.r.get_index_of_closest_point() <NavConstants.FOLLOW_WALL_MIN or self.r.get_index_of_closest_point() > NavConstants.FOLLOW_WALL_MAX:
                        #If it overturns the right angle by +- 5 degrees it turns right or left, to stay at the right angle.

                        if self.r.get_index_of_closest_point()<NavConstants.FOLLOW_WALL_MIN:
                            self.velocity = NavConstants.STOP
                            self.rotation = NavConstants.NORMAL_TURN_RIGHT

                        elif self.r.get_index_of_closest_point()>NavConstants.FOLLOW_WALL_MAX:
                            self.velocity = NavConstants.STOP
                            self.rotation = NavConstants.NORMAL_TURN_LEFT

                    #If the robot is in between the threshold, he drives forward.
                    elif self.r.get_index_of_closest_point() > NavConstants.FOLLOW_WALL_MIN or self.r.get_index_of_closest_point() < NavConstants.FOLLOW_WALL_MAX:
                        self.velocity = NavConstants.NORMAL_VEL
                        self.rotation = NavConstants.DONT_TURN

            #If there is no obstacle that obstructs the robot, he goes into the driving state.
                else:
                    #If the angle between the goal angle and the robots angle are bigger than the max threshold degree, he turns left
                    if math.degrees(self.r.angleToGoal()-self.r.angleOfRobot()) > NavConstants.ANGLE_THRESHOLD_MAX:
                        self.velocity = NavConstants.SLOW_VEL
                        self.rotation = NavConstants.NORMAL_TURN_LEFT

                    #If the angle between the goal angle and the robots angle are smaller than the minimum threshold degree, he turns right
                    elif math.degrees(self.r.angleToGoal()-self.r.angleOfRobot()) < NavConstants.ANGLE_THRESHOLD_MIN:
                        self.velocity = NavConstants.SLOW_VEL
                        self.rotation = NavConstants.NORMAL_TURN_RIGHT

                    #If the robot drives straight towards the goal it is checked if the goal is near, so the robot can slow down,
                    # if not, the robot drives straight.
                    else:
                        if self.r.goalDistance() < NavConstants.NEAR_GOAL:
                            self.velocity = NavConstants.SLOW_VEL
                            self.rotation = NavConstants.DONT_TURN
                        else:
                            self.velocity = NavConstants.NORMAL_VEL
                            self.rotation = NavConstants.DONT_TURN


            self.orders.linear.x = self.velocity
            self.orders.angular.z = self.rotation
            self.pub.publish(self.orders)




#Function that checks if on the index in direction of the goal is any obstacle cloeser than one meters.
## Return value: Returns True when there is an osbtacle and fals if there isn't one.
    def isObstacle(self):
        if (self.r.lScan[int(round(math.degrees(self.r.angleToGoal()-self.r.angleOfRobot())))] < NavConstants.OBSTACLE_DISTANCE or
        self.r.lScan[int(round(math.degrees(self.r.angleToGoal()-self.r.angleOfRobot())))+1] < NavConstants.OBSTACLE_DISTANCE or
        self.r.lScan[int(round(math.degrees(self.r.angleToGoal()-self.r.angleOfRobot())))-1] < NavConstants.OBSTACLE_DISTANCE or
        self.r.lScan[int(round(math.degrees(self.r.angleToGoal()-self.r.angleOfRobot())))+2] < NavConstants.OBSTACLE_DISTANCE or
        self.r.lScan[int(round(math.degrees(self.r.angleToGoal()-self.r.angleOfRobot())))-2] < NavConstants.OBSTACLE_DISTANCE):

            return True
