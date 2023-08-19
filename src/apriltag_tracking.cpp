#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <Eigen/Dense>

mavros_msgs::PositionTarget pose_vel;
apriltag_ros::AprilTagDetectionArray at_in_data;
geometry_msgs::PoseStamped lpp_data;
Eigen::Vector4d u(4);
unsigned previous_at_in_seq{0};


bool lpp_data_in = 0;
bool at_in = 0;
bool all_in = 0;


void at_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& at_msg){
    ROS_INFO_STREAM("Callback function was called!");
    at_in_data = *at_msg; 
    at_in = 1;
}

void lpp_callback(const geometry_msgs::PoseStamped::ConstPtr& lpp_msg) {
    lpp_data = *lpp_msg;
    lpp_data_in = 1; 
}

/**
 * r_PO_I is a 4x1 vector. The first 3 elements of which are the position of the april tag in the inertial frame
 * r_DP_I is a 4x1 vector. The first 3 elements of which are the desired position between the drone and the april tag in the inertial frame
 */
Eigen::Vector4d control_algorithm(const Eigen::Vector4d r_DP_I, const Eigen::Vector4d r_PO_I) {
    return (r_DP_I + r_PO_I);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_tracking_node");
    ros::NodeHandle nh;
    

    ros::Publisher local_pos_pub_mavros = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);
    ros::Subscriber at_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/iris/usb_cam/tag_detections", 1, at_cb);
    ros::Subscriber local_info_sub = nh.subscribe <geometry_msgs::PoseStamped> ("/mavros/local_position/pose", 10, lpp_callback);
    ros::Publisher target_body_pub = nh.advertise<geometry_msgs::PoseStamped>("/tag_detections/tagpose_body", 10);
    ros::Publisher target_lpp_pub = nh.advertise<geometry_msgs::PoseStamped>("/tag_detections/tagpose_inertial", 10);
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    mavros_msgs::PositionTarget pose_vel;
    // Note that this type_mask is assuming you are affecting VELOCITY control over the vehicle.  See the original offboard example for the position type mask
    //pose_vel.coordinate_frame = pose_vel.FRAME_BODY_NED;//pose_vel.FRAME_LOCAL_NED;
    pose_vel.coordinate_frame = 1;
    pose_vel.yaw = 3.14159/2;
	pose_vel.type_mask = pose_vel.IGNORE_VX | pose_vel.IGNORE_VY | pose_vel.IGNORE_VZ | pose_vel.IGNORE_AFZ | pose_vel.IGNORE_AFY | pose_vel.IGNORE_AFX;

    ros::Time last_request = ros::Time::now();

    int count = 0;

    while(ros::ok()) {

        ros::spinOnce();
        if (at_in_data.detections.empty()) {
        ROS_WARN_STREAM("No AprilTags detected.");
        // Here you could add any fallback behavior if necessary.
        // current one is just to hover in place. 
        pose_vel.header.stamp = ros::Time::now();
        pose_vel.position.x = lpp_data.pose.position.x;
        pose_vel.position.y = lpp_data.pose.position.y;
        pose_vel.position.z = 2.0; // in meters in local inertial frame
        local_pos_pub_mavros.publish(pose_vel);

        continue; // Skip the rest of this iteration of the loop.
    }


        if (previous_at_in_seq != at_in_data.header.seq) {
            previous_at_in_seq = (unsigned)at_in_data.header.seq;
            count = 0;
            ROS_INFO_STREAM("SEEN APRILTAG!!");
        } else {
            ++count;
        }

        if (count == 20) {
            // recall rate is 20 Hz, therefore, 100 cycles is equivalent to 100 / 20 = 5 seconds
            at_in = 0;
        }

        if(at_in && lpp_data_in){
            all_in = 1;
        } else {
            all_in = 0;
        }

        
        // The position of the drone relative to the local inertial frame (vehicle starts along y axis using indoor parameters)
        // Drone axes are x = forward, y = left, z = up (FLU)
        // Vehicle should start with initial orientation of 90 deg right; quaternion = (0,0,-0.707, -0.707)
        double xp = lpp_data.pose.position.x;
        double yp = lpp_data.pose.position.y;
        double zp = lpp_data.pose.position.z;

        // Position of the apriltag in the camera coordinate frame.  Z coincident with optical axis; Y down in camera frame; X to the right when looking from vehicle out
        double xt = at_in_data.detections[0].pose.pose.pose.position.x;
        double yt = at_in_data.detections[0].pose.pose.pose.position.y;
        double zt = at_in_data.detections[0].pose.pose.pose.position.z;

        // EIGEN's convention is to have the SCALAR value FIRST!!!
        Eigen::Quaterniond quat_lpp(lpp_data.pose.orientation.w, lpp_data.pose.orientation.x, lpp_data.pose.orientation.y, lpp_data.pose.orientation.z);
        Eigen::Matrix3d R_lpp = quat_lpp.toRotationMatrix();

        // Populating a homogenous transformation matrix with /mavros/local_position/pose data
        // converting quaternion to rotation matrix
        Eigen::Matrix4d H_lpp;
        H_lpp.block(0,0,4,4) = Eigen::Matrix4d::Constant(4,4, 0.0);
        H_lpp.block(0,0,3,3) = R_lpp;
        H_lpp(3,3) = 1.0;
        H_lpp(0,3) = xp;
        H_lpp(1,3) = yp;
        H_lpp(2,3) = zp;

        // A fixed homogenous transformation (zero translation) to convert between camera frame and local body FLU frame
        // Camera is pointing 45 deg down looking forward. = tracking camera
        // This matrix takes points in the body frame and converts to camera frame
        Eigen::Matrix4d H_M_B;
        H_M_B << 0,-1,0,0,-0.707,0,-0.707, 0, 0.707, 0, -0.707, 0, 0, 0, 0, 1;

        // create a vector with apriltag position relative to camera
        Eigen::Vector4d r4vec(4);
        r4vec << xt,yt,zt,1;

        // Rotate the apriltag position from camera coordinates to FLU coordinates
        // inverse H_M_B converts from camera to body coordinates.
        // Note: For unitary transformation (unitary rotation matrix) transpose of matrix = inverse of matrix
        Eigen::Vector4d P_r_B(4);
        P_r_B = H_M_B.inverse()*r4vec;

        Eigen::Vector4d P_r_I(4);
        P_r_I = H_lpp*P_r_B;

        // This computes P_r_I2 which is the computed location of the Apriltag in the local inertial mavros lpp frame
        Eigen::Vector4d P_r_I2(4);
        Eigen::Matrix4d H_lpp_nopos;
        H_lpp_nopos = H_lpp;
        H_lpp_nopos(0,3) = 0;
        H_lpp_nopos(1,3) = 0;
        H_lpp_nopos(2,3) = 0;
        P_r_I2 = H_lpp_nopos*H_M_B.inverse()*r4vec;

        // This computes the difference between the inertial location of the apriltag and inertial location of the vehicle
        Eigen::Vector4d I_diff;
        I_diff = P_r_I2-P_r_I;

        // This computes the Euler angles associated with the mavros/local_position/pose quaternion in yaw->pitch->roll format.
        Eigen::Vector3d euler_ang = quat_lpp.toRotationMatrix().eulerAngles(2,1,0);

        // This creates a rotation matrix considering ONLY the heading (not roll/pitch) so we can rotate between body and inertial heading
        // I'm mentally thinking of this as body referenced, but "stabilized" by removing roll and pitch 
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix << cos(euler_ang(0)), -sin(euler_ang(0)), 0, sin(euler_ang(0)),  cos(euler_ang(0)), 0, 0, 0, 1;


        // populate and publish the apriltag in body FLU coordinates
        geometry_msgs::PoseStamped body_pub_data;
        body_pub_data.header.stamp = lpp_data.header.stamp;
        body_pub_data.pose.position.x = P_r_B(0);
        body_pub_data.pose.position.y = P_r_B(1);
        body_pub_data.pose.position.z = P_r_B(2);
        target_body_pub.publish(body_pub_data);

        // populate and publish the apriltag in inertial coordinates
        geometry_msgs::PoseStamped lpp_pub_data;
        lpp_pub_data.header.stamp = lpp_data.header.stamp;
        lpp_pub_data.pose.position.x = P_r_I(0);
        lpp_pub_data.pose.position.y = P_r_I(1);
        lpp_pub_data.pose.position.z = P_r_I(2);
        target_lpp_pub.publish(lpp_pub_data);


        Eigen::Vector4d r_DP_I(4);
        
        r_DP_I << 0.0, -0.5, 0.0, 1.0;
        
        if (all_in) {
            // all_in is set to true, if, during this iteration of the while loop, we have obtained both a tag_detection measurement and a mavros/local_position/pose measurement
            // - if we don't get tag_detection or mavros data, then don't update the control input, and just repeat the control input requested at the previous time-step

            // P_r_I is the position of the april tag in the inertial frame (i.e. r_PO_I)
            u = control_algorithm(r_DP_I, P_r_I);
        } else {
            // if we reach this, then either we are just starting up, or we haven't seen the april tag in 5 or more seconds (so at least 5 seconds)
            ROS_INFO_STREAM("Haven't seen april tag at least 1 seconds");
        }
        

        pose_vel.header.stamp = ros::Time::now();
        pose_vel.position.x = u(0);//u(1);
        pose_vel.position.y = u(1);//u(0);
        pose_vel.position.z = 1.5; // in meters in local inertial frame
        local_pos_pub_mavros.publish(pose_vel);
        
        // ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


