#include <aruco2_ros/utils.h>

#include <opencv2/calib3d.hpp>
#include <eigen_conversions/eigen_msg.h>

aruco::CameraParameters aruco2_ros::ros_caminfo_to_aruco(const sensor_msgs::CameraInfo& msg) {
    cv::Mat proj(3, 3, CV_32FC1);
    cv::Mat dist(4, 1, CV_32FC1);
    cv::Size size(msg.height, msg.width);

    for (int i = 0; i < 9; i++) {
        proj.at<float>(i%3, i-(i%3)*3) = msg.K[i];
    }

    for (int i = 0; i < 4; i++) {
        dist.at<float>(i, 0) = msg.D[i];
    }

    return aruco::CameraParameters(proj, dist, size);
}

geometry_msgs::Pose aruco2_ros::aruco_marker_to_pose(const aruco::Marker& marker) {
    cv::Mat cv_t = marker.Tvec;
    cv::Mat cv_R(3, 3, CV_32FC1);
    cv::Rodrigues(marker.Rvec, cv_R);

    Eigen::Vector3d transl;
    transl << cv_t.at<float>(0), cv_t.at<float>(1), cv_t.at<float>(2);
    Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> rot_mat(cv_R.ptr<float>(), 3, 3);
    Eigen::Quaterniond rot(rot_mat.cast<double>());

    geometry_msgs::Pose result;
    tf::pointEigenToMsg(transl, result.position);
    tf::quaternionEigenToMsg(rot, result.orientation);

    return result;
}