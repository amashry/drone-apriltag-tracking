// #include "ros/ros.h"
// #include "apriltag_ros/AprilTagDetectionArray.h"

// void detectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
// {
//     for(const auto& detection : msg->detections)
//     {
//         ROS_INFO("Detected AprilTag, ID: %d, Position: (%f, %f, %f)", 
//                 detection.id[0],
//                 detection.pose.pose.pose.position.x, 
//                 detection.pose.pose.pose.position.y, 
//                 detection.pose.pose.pose.position.z);
//     }
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "apriltag_tracking_node");

//     ros::NodeHandle n;

//     ros::Subscriber sub = n.subscribe("/iris/usb_cam/tag_detections", 1000, detectionCallback);

//     ros::spin();

//     return 0;
// }
