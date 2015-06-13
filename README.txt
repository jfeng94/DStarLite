DStarLite README

The algorithm runs in the same package that was distributed for ME/CS132a. Following the same setup you can install by running

git clone https://www.github.com/jfeng94/DStarLite 

in the source code directory in the workspace.

Next, modify the following files:


me132ab_ws/src/me132a_epoch/CMakeLists.txt:
Add the following:
add_executable (D_star_lite src/DStarLite/main.cpp)
add_library(D_star_lite_classes src/DStarLite/map.h src/DStarLite/map.cpp)
target_link_libraries(D_star_lite ${catkin_LIBRARIES} D_star_lite_classes)


After these files are configured, navigate to /me132ab_ws and enter:
catkin_make
After the files make, run: 
roslaunch me132a_epoch stage.launch

Open another terminal, navigate to the same directory, and enter:
rosrun me132a_epoch D_star_lite < input arguments >
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
