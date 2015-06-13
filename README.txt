DStarLite README

The algorithm runs in the same package that was distributed for ME/CS132a. Following the same setup, modify the following files:

me132ab_ws/src/me132a_epoch/CMakeLists.txt:
Add the following:
add_executable (main src/main.cpp)
add_library (map src/map.cpp src/map.h)
target_link_libraries (map {$CATKIN_LIBRARIES})

Add main.cpp, map.cpp, and map.h to me132ab_ws/src/me132a_epoch/src

After these files are configured, navigate to /me132ab_ws and enter:
catkin_make
After the files make, run: 
roslaunch me132a_epoch stage.launch

Open another terminal, navigate to the same directory, and enter:
rosrun me132a_epoch main < input arguments >
where in the input arguments, the following arguments are taken:
xmin ymin xmax ymax res startx starty theta goalx goaly
xmin: lower left corner of the map's x coordinate
ymin: lower left corner of the map's y coordinate
xmax: upper right corner of the map's x coordinate
ymax: upper right corner of the map's y coordinate
res: resolution of the map (block size)
startx: starting x position
starty: starting y position
theta: initial angle
goalx: goal x coordinate
goaly: goal y coordinate
