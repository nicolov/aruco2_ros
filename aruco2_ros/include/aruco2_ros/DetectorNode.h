#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>

#include <aruco/aruco.h>

#include <aruco2_ros/utils.h>
#include <aruco2_msgs/MarkerArray.h>

namespace aruco2_ros {
    struct DetectorNode {
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber img_sub_;
        ros::Publisher markers_pub_;
        tf::TransformBroadcaster tfbr_;

        double marker_size_;
        aruco::MarkerDetector detector_;
        aruco::CameraParameters camera_params_;
        std::vector<aruco::Marker> markers_;

        DetectorNode() : nh_("~"),
                         it_(nh_) {
            nh_.param<double>("marker_size", marker_size_, 0.1);
            ROS_INFO_STREAM("Using marker size: " << marker_size_);

            img_sub_ = it_.subscribe("/camera/image_mono", 1, &DetectorNode::image_callback, this);

            ROS_INFO_STREAM("Waiting for calibration...");
            auto caminfo_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/camera_info", nh_);
            camera_params_ = ros_caminfo_to_aruco(*caminfo_msg);
            ROS_INFO_STREAM("Received calibration");

            markers_pub_ = nh_.advertise<aruco2_msgs::MarkerArray>("markers", 10);

            // Use the recommended marker dictionary (see aruco/dictionary.h)
            detector_.setDictionary(aruco::Dictionary::DICT_TYPES::ARUCO_MIP_36h12);
        }

        void image_callback(const sensor_msgs::ImageConstPtr& msg) {
            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            markers_.clear();
            detector_.detect(cv_ptr->image, markers_, camera_params_, marker_size_, false);

            {
                aruco2_msgs::MarkerArray markers_msg;
                markers_msg.header = msg->header;
                for (auto& m: markers_) {
                    aruco2_msgs::Marker mrkr;
                    mrkr.header = msg->header;
                    mrkr.id = m.id;
                    mrkr.confidence = 1.0;
                    mrkr.pose.pose = aruco_marker_to_pose(m);
                    markers_msg.markers.push_back(mrkr);

                    // Publish tf
                    tf::Transform tft;
                    std::ostringstream label_ss;
                    label_ss << "m_" << mrkr.id;
                    tf::poseMsgToTF(mrkr.pose.pose, tft);
                    tfbr_.sendTransform(
                            tf::StampedTransform(tft, msg->header.stamp, "camera", label_ss.str()));
                }

                markers_pub_.publish(markers_msg);
            }
        }
    };
}