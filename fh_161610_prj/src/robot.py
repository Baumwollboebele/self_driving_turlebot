
import rospy
import math
from sensor_msgs.msg import LaserScan
from goal_publisher.msg import PointArray
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from array import *


#The class implements functions which are responsible for calculating and processing sensor data
class Robot(object):

    goalSubscriber = None

    currentGoal = 0
    maxGoals = 0

    posGoal = None
    posSelf = None

    quaternion = None

    lScan = None

    xAxis = [1,0]
    yAxis = [0,1]

# Initialises the subscribers.
# The goal subscriber is stored in a variable, so it can easily be unsibscribed as soon as it is initialized,
# due to some false values it can contain.

    def __init__(self):
        goalSubscriber = rospy.Subscriber('/goals', PointArray,self.goalCallback)
        rospy.Subscriber('/gazebo/model_states',ModelStates,self.modelCallback)
        rospy.Subscriber('/scan',LaserScan,self.laserCallback)

# Callback function that stores the three goal's position coordinates in an 2-dimensional Array
    def goalCallback(self,goal):
        self.posGoal = [[0,0],[0,0],[0,0]]
        self.posGoal[0][0] = goal.goals[0].x
        self.posGoal[0][1] = goal.goals[0].y

        self.posGoal[1][0] = goal.goals[1].x
        self.posGoal[1][1] = goal.goals[1].y

        self.posGoal[2][0] = goal.goals[2].x
        self.posGoal[2][1] = goal.goals[2].y

# Callback function that stores the robot-models position coordinates in an array.
# It also gets the orientation values and stores them in an seperate array.
    def modelCallback(self,model):
        self.posSelf = [0,0]
        self.posSelf[0] = model.pose[1].position.x
        self.posSelf[1] = model.pose[1].position.y

        self.quaternion = [0,0,0,0]
        self.quaternion[0] = model.pose[1].orientation.x
        self.quaternion[1] = model.pose[1].orientation.y
        self.quaternion[2] = model.pose[1].orientation.z
        self.quaternion[3] = model.pose[1].orientation.w

# Callback function that stores the values of the laser ranges in an array:
    def laserCallback(self,scan):
        self.lScan = scan.ranges

#Function calculates the vector from the robot's position to the goal's by subtracting
## Return values: Int Array where [0] index is the x coordinate of the vector and [1] is the y coodrinate
    def vectorToGoal(self):
        if self.valuesInitiated() is True:
            vtg = [((self.posGoal[self.currentGoal][0])-(self.posSelf[0])),((self.posGoal[self.currentGoal][1])-(self.posSelf[1]))]
            return vtg
        else:
            print "vectorToGoal: Callback values not initiated!"


#Function calculates the goal vector's length by by calculating his magnitude.
##Return values: Float value of the vectors length in meters
    def goalDistance(self):
        if self.valuesInitiated() is True:
            return math.sqrt(self.vectorToGoal()[0]**2+self.vectorToGoal()[1]**2)
        else:
            print "goalDistance: Callback values not initiated"

#Get function for the laser data
##Return values: Array with all the distances from index 0 to 359, indexes corrospond to the degree of the robot counter clockwise.
    def getLaserData(self):
        if self.valuesInitiated is True:
            return self.lScan.ranges
        else:
            print "getLaserData: Callback values not initiated!"


#Function that calculates the angle of the goal vector with the x axis
##Return values:returns a float
    def angleToGoal(self):
        if self.valuesInitiated() is True:
            return math.atan2(self.vectorToGoal()[1],self.vectorToGoal()[0])
        else:
            print "angleOfGoal: Callback values not initiated!"


#Function that calculates the angle of the robot with the x axis with quaternion
##Return values: Float array, where the index [0] is the x coordinate and the [1] index is the y coordinate
    def angleOfRobot(self):
        if self.valuesInitiated() is True:
            return euler_from_quaternion([self.quaternion[0],self.quaternion[1],self.quaternion[2],self.quaternion[3]])[2]
        else:
            print "angle of robot: Callback values not initiated!"


#Get function for the current goal
##Return value: Int
    def getCurrentGoal(self):
        return self.currentGoal

#Function that increments the curent goal
##Parameters: Int of how much the goals will be incremented.
    def incCurrentGoal(self,value):
        self.maxGoals = self.maxGoals + 1

        if self.maxGoals == 3:
            print "Reached all goals"
            rospy.signal_shutdown("")
        else:
            self.currentGoal = self.currentGoal + value

#Function that checks if all the callback values are initiated
##Return value: True, if all values are initiated
    def valuesInitiated(self):
        if self.posGoal != None and self.posSelf != None and self.quaternion != None and self.lScan != None:
            if self.goalSubscriber != None:
                self.goalSubscriber.unsubscribe()
            return True


#Function returns the index of the closest point in the laser scan array
## Return value: Int of the index in the array
    def get_index_of_closest_point(self):
        if self.valuesInitiated() is True:
            min_value=8
            min_index=0

            for index in range(len(self.lScan)):
                if self.lScan[index] != 0:
                    if self.lScan[index] < min_value:
                        min_value = self.lScan[index]
                        min_index = index
            return min_index
