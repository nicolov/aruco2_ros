aruco2_ros
==========

A set of ROS packages to simplify using the [ArUco](http://www.uco.es/investiga/grupos/ava/node/26) fiducial markers library (version 2.x). I've found the 2.x branch to be great, as it reliably supports many families of markers and compiles easily using OpenCV3.

Included packages
-----------------

- **aruco2_catkin**, a simple catkin wrapper that downloads the library from the authors' website, compiles it, and makes the headers and libraries available as a catkin package;

- **aruco2_ros**, an example of a very basic detector node, that publishes information about each detected marker using the message types defined in **aruco2_msgs**. I've kept this very minimal, see Philosophy below;

- **aruco2_gazebo_model_example**, a ready-to-use 3d model for the gazebo simulator. The *m1* models is a little square with the ArUco marker id=1 from the `ARUCO_MIP_36h12` dictionary (as recommended by the authors for maximum reliability). It's plug-and-play with the included launch file that spawns the model into whatever world you have running.

Getting started
---------------

You can download an example dataset [here](https://db.tt/qdOj3keW) and use the provided launch file to start the detector

```
cd ~/catkin_ws/src
git clone github.com/nicolov/aruco2_ros
catkin build aruco2_ros

# Start the detector
roslaunch aruco2_ros detector_test.launch

# Start playing the demo bag (on a different terminal)
rosbag play ~/Downloads/aruco2_ros_demo_bag.bag

# Visualize the transforms using rviz
rviz
```

Philosophy
----------

Compared to other fiducial marker libraries available for ros, this package:

- leverages catkin and CMake for a clean build process that downloads the ArUco source without duplicating it in the repo. Also, there are no modifications to upstream, to avoid hard-to-trace bugs;

- only offers a minimal detector node to show the basic ArUco API calls. In the past, I've found kitchen-sink nodes confusing to understand and modify, and incomplete anyway.

License
-------

MIT