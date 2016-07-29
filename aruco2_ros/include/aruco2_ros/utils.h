#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <aruco/aruco.h>


namespace aruco2_ros {
    aruco::CameraParameters ros_caminfo_to_aruco(const sensor_msgs::CameraInfo &msg);

    geometry_msgs::Pose aruco_marker_to_pose(const aruco::Marker& marker);
}