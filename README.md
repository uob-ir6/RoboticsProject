## steps to run
### prequisites
1) git clone this repository

### steps
1) copy the package folder to your catkin_ws/src folder
2) rename the package folder to "waiter_robots"
3) cd to scripts/node.py and run
```
chmod +x node.py
```
this makes the file executable 
4) in catkin_ws run
``` 
catkin_make
```
5) to run the code now run in seperate terminals
```
roscore
rosrun stage_ros stageros "path to map world file"
rosrun map_server map_server "path to map yaml file"
rosrun rviz rviz
rosrun waiter_robots node.py
```
