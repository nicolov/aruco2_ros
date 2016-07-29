#include <aruco2_ros/DetectorNode.h>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "aruco2_detector_node");
    aruco2_ros::DetectorNode node;

    ros::spin();
}
