# offboard
## Installation
(in your current working directory)  
**1. create a catkin workspace**  
    mkdir -p catkin_ws/src  
**2. cd to src**  
    cd catkin_ws/src  
    git clone https://github.com/giangthewalkingman/offboard.git  

## Usage
**3. open CMakeLists.txt to uncomment the code block in "for build on your laptop" and comment the code block in "for build on rpi server" (if you are using your laptop), if you are using the rpi server, do the opposite**  
    sudo nano offboard/CMakeLists.txt  
**4. back to the workspace and build the code**  
    cd ..  
    catkin_make  
**5. run the code**  
    source ./devel/setup.bash  
    roslaunch offboard offboard.launch  (in terminal 1)  
    roslaunch setmode setmodeoffb       (in terminal 2)  

## Warnings
* At this time, the code cannot run on your laptop as simulation because the files are specificially targetted to run on the raspberry pi.
* On the raspberry pi, the code works well

## Versions
* raspberry pi: 4b, 1gb ram, ubuntu server 20.04
* ros: noetic
