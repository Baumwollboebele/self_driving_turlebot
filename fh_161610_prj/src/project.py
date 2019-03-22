#!/usr/bin/env python

import rospy
import time
from robot import Robot
from navigator import Navigator


rospy.init_node('project')


#Main that starts the whole project, by creating the needed objects and calling the functions.

def main():


    robot = Robot()

    navigator = Navigator(robot)

    navigator.drive()



if __name__ == "__main__":
    main()


rospy.spin()
