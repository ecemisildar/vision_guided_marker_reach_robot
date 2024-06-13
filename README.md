# Assignment1 Experimental robotics laboratory
## Group Members | Name-Surname & GitHub Account
* Isabel Cebollada Gracia | [@isacg5](https://github.com/isacg5)
* Ecem Isildar | [@ecemisildar](https://github.com/ecemisildar)
* Baris Aker | [@barisakerr](https://github.com/barisakerr)
* Carmine Miceli | [@Carmine00](https://github.com/Carmine00)
* Fabrizio Francesco Leopardi | [@FabrizioLeopardi](https://github.com/FabrizioLeopardi)

## Brief description of the project
The aim of this project is to use a robot with a camera mounted on it to detect the four markers and reach them when at least 200 pixels are detected by the camera. The world environment and the robot model are already given. Therefore, a controller node is written then simulation and real-world tests are done successfully by the group members.

## How to use?
For the execution of this project, is necessary to clone the repository in the ros workspace. After catkin_make it, it is only necessary to run the following command:
```console
roslaunch assignment1_EXP sim_aruco.launch
```

## Brief description of the code
In this assignment, a mobile robot starting from (0,0) detected the four markers in the environment. Using the Aruco ros library and Marker message and the camera of the robot, each marker corner is detected then the middle point of the markers is found and with the help of the state machine structure of the controller node, all markers are reached respectively. In order to find the marker angular velocity was given to the robot, after the middle point of the marker was detected using linear velocity the robot reached the marker until it reached all the markers.

## Flowchart
<p align="justify">
  The flowchart of the controller and the marker publisher is shown on the left and right sides respectively
</p>

[![Flowchart](https://github.com/isacg5/assignment1_EXP_rep/blob/main/resources/flowchart.png)](https://github.com/isacg5/assignment1_EXP_rep/blob/main/resources/flowchart.png)

## Possible Improvements 
* The movement of the robot in the simulation can be improvable. Instead of a sudden behavior of the robot at the beginning, the movement of the robot could start with a slower velocity. With this improvement, more smoother movement behavior could be observable.

* When the project is tested on the real robot, the delay, which does not exist in the simulation, can be visible, during the rotation of the robot to detect the markers. This delay may be due to the data-sending speed, the delay can be hidden by reducing the robot's angular velocity from the controller.

## Videos
<p align="justify">
  Watch the video using the simulated robot robot here
</p>

[![Watch the video](https://github.com/isacg5/assignment1_EXP_rep/blob/main/resources/sim_robot.png)](https://youtu.be/M5vqtRnRwP8)





