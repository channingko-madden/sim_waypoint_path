# sim_waypoint_path
A ROS 2 package with tools for creating and visualizing waypoint paths.

## Setup
This package was testing using ROS 2 Foxy on Ubuntu 20.04.5

### ROS2 Build
This repo is set up as a ROS 2 package. When placed within a standard ROS 2 workspace
(ex. /dev_ws/src/sim_waypoint_path), it can be built by calling `colcon build` from the 
root of the workspace, after which `source install/setup.bash` must also be called so that the ros2 run
tool can find the executables.

These steps are required to be able to run the ROS scripts in this repo.

### Install Python package 
To install this as a normal python package, run `python3 setup.py install` (may require sudo)

## Unit Tests
Pytest is used for running unit tests, which are located in the [test](test) folder.
You can run all the unit tests at once by calling `python3 -m pytest test/` from the base directory of this repo.\
You can run individual units tests by calling `python3 -m pytest test/test_example.py` as well.

## Scripts

### rviz_point_visualizer
This script has required arguments for the number of sides and length that form a regular polygon
(a polygon that is both equilateral and equiangular), as well as an optional flag argument that indicates 
an initial pose should be obtained using tf.\
Using the sides, length, and an initial pose, it generates a list of poses that form the polygon, and 
publishes these poses to RViz as Markers.

See [here](sim_waypoint_path/pose_generators.py) to find the function that creates the poses.

#### Setup
- Make sure RViz is up and running. By default there should be a global fixed frame
named "map". This is the frame that the markers will be published to.
- Within RViz, choose to add a visualization by topic, which will list the /visualization_marker topic as
an option too add.
- If using tf2 to set an initial pose, make sure there is a transform from the "map" to "base_link" frame available.
This can be done using the static_transform_publisher tool provided by tf2.\
(For example, running `ros2 run tf2_ros static_transform_publisher 2 2 0 0 0 0 1 map base_link` will set the
initial pose at an (x,y,z) position of (2,2,0).

#### Running the script
- To run with a default initial pose at the origin of the map frame:\
`ros2 run sim_waypoint_path rviz_point_visualizer -s sides -l length`
 - where `sides` is the number of sides and `length` is the length of all sides of the polygon.
- To run using tf2 to obtain an initial pose:\
`ros2 run sim_waypoint_path rviz_point_visualizer -s sides -l length -tf`
- Information on the cli arguments can be obtain using:\
`ros2 run sim_waypoint_path rviz_point_visualizer --help`
