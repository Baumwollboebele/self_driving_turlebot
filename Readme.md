# AMR: Final bachelor project documentation


## Directory structure
This repository contains a ros program, the solution to the final project and a gazebo model, which is used as the final projects test environment.




The project has the following directory structure:

```
.
├── fh_161610_prj
│   ├── CMakeLists.txt
│   ├── launch
│   │   └── start.launch
│   ├── package.xml
│   └── src
│       ├── navigator.py
│       ├── project.py
│       ├── robot.py
├── final_project_practice
│   └── model.sdf
├── Readme.md
└── Presentation.md
```
<br>
<hr>
<br>

## How to start the project


The robot can be startetd by using following commands:

- Before you can get started make sure that all your **catkin packages are build**, by going in the catkin folders root directory and type following command:

    ```
    catkin_make
    ```


- Open the console and **start roscore** with the command:
   
   ```
    roscore
   ```

- Type in the next command which will **start** the **gazebo** environment and spawn the turtlebot in an empty world:
  
    ```
    roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
    ```

-  You can **generate the** final projects **environment** with the command:
     ```
    rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/fh-161610_tier4/final_project_practice/model.sdf -sdf -x 2 -y 1 -model mini_project
    ```

- Now you can finally **start the programm** that starts the robot:
    ```
    roslaunch fh_161610_prj start.launch
    ```
<br>
<hr> 
<br>

## The robot
### Task:
The robot finds itself in a room with some obstacles and is given the coordinates of three goals.
The aim is, to drive sequentially towards each goal and drive around the obstacles in the way.
The project has to be written in python and it was allowed to call the ros services and topics given.

![](/Ressources/world.png)

### Project structure
The project consists of the classes:
- Navigator
- Project
- Robot
- NavConstants

The Project class is the main class and creates all the needed objects.

The Robot class provides all the data that is collected from the robots sensors, it also evaluates the raw data and has functions that make this data usable.

The Navigator class is the robots logic, it decides the path the robot takes.

The NavConstants class holds constant values, which are used by the robot.

<br>

## Topics 
For this project I used the following topics:

<br>

### Subscriber
`/goals`: 
- publishes an PointArray of the goals


`/gazebo/model_states`:
- publishes data concerning the objects, like coordinates.
  

`/scan`:
- publishes the laserdata within an array from 0 to 360 which also represents the angle from the robots front counter clockwise.




### Publisher
`/cmd_vel`:
- publishes the velocity and turning speed of the robot.


<br>
<hr> 
<br>


## The algorithm

The algorithm is implemented as a state machine, the robot knows the 3 states:
- Goal reached
- Obstacle detected
- Drive

The **states** are **checked** **sequentially**, beginning with the "goal reached" state, then the "obstacle detected" state and at last the "drive" state.
The overall algorithm is **going in a loop until the last goal is reached.**
Before entering the driving algorithm, it is **checked**  by the function `isInitialised()` **if**   all the necessary **data is initialised**.

![](/Ressources/flowchart.png)


<br>

### Goal reached:
In this state it is constantly checked whether the function `goalDistance()` which returns the length of the goal vector, is smaller than the distance you are allowed to be away from the goal.

![](/Ressources/goal.gif)

The robot then prints the current goal into the console, stops for 5 seconds and sets the next goal as the current goal.
If there is no more goal, the robot stops and the rospy node gets shutdown.

<br>

### Obstacle detected:

In this state it is checked, in the function `isObstacle()`, if the distance of the array index of the laser data, in direction of the goal vector is smaller than the minimum distance set.

If this is the case, then it is checked if the closest point is between 85° and 95°.

If this is not the case, the robot will turn until it is at an approximat 90° angle to the obstacle.

It then drives, while constantly checking if the sensor data from the angle where the goal vector points to is not obstructed.


![](/Ressources/obstacle.gif)

<br>

### Drive

In this state, it is checked if the difference between the angle of the robots orientation vector and the goal vector are positiv or negativ.

If the angle between those two vectors is negative and not in the threshold defined in the constants class, the robot will turn right.
If the angle is positiv and not within the threshold it will turn left.

![](/Ressources/winkel.png)

If the robot drives within this threshold the robot will drive forward, he then cheks constantly, in the function `goalDistance()` whether he is close to the goal or not.
If he is close to the goal he will slow down.

<br>
<hr>
<br>

## Difficulties

A problem I encountered a lot, was that if you use just one return value of the laser data, the robot often will overturn or even bump into obstacles.

![](/Ressources/close.png)

As you can see in the picture above, the robot would crash against the wall, even though the vector to the goal would not be obstructed.



### Solution

The solution was quiet easy. I assigned thresholds for such problems, big enough to let the robot pass by objects easily, but small enough to get stuck in small entrances.

![](/Ressources/notclose.png)



