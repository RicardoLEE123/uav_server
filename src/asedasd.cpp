#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <thread>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/HilSensor.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Imu.h>

// sub /mavros/log_transfer/raw/log_data

// pub /mavros/hil/imu_ned
// pub /mavros/odometry/out
// pub /mavros/setpoint_raw/local

ros::Publisher imu_ned_pub;
ros::Publisher odometry_pub;
ros::Publisher setpoint_raw_pub;

void visionCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    nav_msgs::Odometry odom_stm32;
    odom_stm32 = *odom;
    odom_stm32.header.frame_id = "odom_ned";
    odom_stm32.child_frame_id = "base_link_frd";
    odometry_pub.publish(odom_stm32);
}

void poscmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& pos_cmd)
{
    mavros_msgs::PositionTarget postarger;
    postarger.header = pos_cmd->header;
    postarger.coordinate_frame = 1;
    postarger.type_mask = 0;
    postarger.position = pos_cmd->position;
    // 2 1 -3
    postarger.position.x = pos_cmd->position.y;
    postarger.position.y = pos_cmd->position.x;
    postarger.position.z = -pos_cmd->position.z;
    
    postarger.velocity = pos_cmd->velocity;
    postarger.velocity.x = pos_cmd->velocity.y;
    postarger.velocity.y = pos_cmd->velocity.x;
    postarger.velocity.z = -pos_cmd->velocity.z;
    
    postarger.acceleration_or_force = pos_cmd->acceleration;
    postarger.yaw = (float) pos_cmd->yaw;
    postarger.yaw_rate = (float) pos_cmd->yaw_dot;
    setpoint_raw_pub.publish(postarger);
}

void imurawCallback(const sensor_msgs::Imu::ConstPtr& imu_raw)
{
    mavros_msgs::HilSensor hilsensor_data;
    hilsensor_data.gyro = imu_raw->angular_velocity;
    hilsensor_data.gyro.x = imu_raw->angular_velocity.x;
    hilsensor_data.gyro.y = -imu_raw->angular_velocity.y;
    hilsensor_data.gyro.z = -imu_raw->angular_velocity.z;
    hilsensor_data.acc = imu_raw->linear_acceleration;
    hilsensor_data.acc.x = imu_raw->linear_acceleration.x;
    hilsensor_data.acc.y = -imu_raw->linear_acceleration.y;
    hilsensor_data.acc.z = -imu_raw->linear_acceleration.z;
    hilsensor_data.header = imu_raw->header;
    hilsensor_data.header.frame_id = "bask_link"; //

    imu_ned_pub.publish(hilsensor_data);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "to_stm32");
    ros::NodeHandle nh;

    imu_ned_pub = nh.advertise<mavros_msgs::HilSensor>("/mavros/hil/imu_ned", 10);
    odometry_pub = nh.advertise<nav_msgs::Odometry>("/mavros/odometry/out", 10);
    setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    ros::Subscriber imu_raw_sub = nh.subscribe("/imu_data_filterd", 1, imurawCallback);
    ros::Subscriber odom_sub = nh.subscribe("/ov_msckf/odomimu", 1, visionCallback);
    ros::Subscriber pos_cmd_pub = nh.subscribe("/drone_0_planning/pos_cmd", 1, poscmdCallback);

    ros::spin();

    return 0;
}







// ros::Publisher odom_pub[4];

// tf::StampedTransform imu_cam_transform[4];
// tf::Transform local_pose;

// const int process_every_nth_frame_ = 2;
// static int frame_counter_ = 0;
// void visionCallback(const nav_msgs::Odometry::ConstPtr& odom)
// {
//     if (++frame_counter_ < process_every_nth_frame_) {
//         return;
//     }
//     frame_counter_ = 0;
//     geometry_msgs::Pose odom_pose = odom->pose.pose;
//     tf::poseMsgToTF(odom_pose, local_pose);

//     for (int i = 0; i < 4; i++) {
//         geometry_msgs::PoseStamped pose_msg;
//         tf::Transform global_cam_transform = local_pose*imu_cam_transform[i];
//         pose_msg.header.stamp = odom->header.stamp;
//         pose_msg.header.frame_id = "global";
//         pose_msg.pose.position.x = global_cam_transform.getOrigin().x();
//         pose_msg.pose.position.y = global_cam_transform.getOrigin().y();
//         pose_msg.pose.position.z = global_cam_transform.getOrigin().z();
//         pose_msg.pose.orientation.x = global_cam_transform.getRotation().x();
//         pose_msg.pose.orientation.y = global_cam_transform.getRotation().y();
//         pose_msg.pose.orientation.z = global_cam_transform.getRotation().z();
//         pose_msg.pose.orientation.w = global_cam_transform.getRotation().w();

//         odom_pub[i].publish(pose_msg);
//     }
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "tf_to_odom_publisher");
//     ros::NodeHandle nh;

//     odom_pub[0] = nh.advertise<geometry_msgs::PoseStamped>("/seeker/camera_pose", 10);
//     odom_pub[1] = nh.advertise<geometry_msgs::PoseStamped>("odom_cam2", 10);
//     odom_pub[2] = nh.advertise<geometry_msgs::PoseStamped>("odom_cam4", 10);
//     odom_pub[3] = nh.advertise<geometry_msgs::PoseStamped>("odom_cam6", 10);
//     tf::TransformListener listener;

//     while (ros::ok()) {
//         try {
//             listener.lookupTransform("imu", "cam0", ros::Time(0), imu_cam_transform[0]);
//             break;
//         }
//         catch (tf::TransformException &ex) {
//             ROS_ERROR("%s", ex.what());
//             ros::Duration(0.2).sleep();
//             continue;
//         }
//     }
//     while (ros::ok()) {
//         try {
//             listener.lookupTransform("imu", "cam2", ros::Time(0), imu_cam_transform[1]);
//             break;
//         }
//         catch (tf::TransformException &ex) {
//             ROS_ERROR("%s", ex.what());
//             ros::Duration(0.2).sleep();
//             continue;
//         }
//     }
//     while (ros::ok()) {
//         try {
//             listener.lookupTransform("imu", "cam4", ros::Time(0), imu_cam_transform[2]);
//             break;
//         }
//         catch (tf::TransformException &ex) {
//             ROS_ERROR("%s", ex.what());
//             ros::Duration(0.2).sleep();
//             continue;
//         }
//     }
//     while (ros::ok()) {
//         try {
//             listener.lookupTransform("imu", "cam6", ros::Time(0), imu_cam_transform[3]);
//             break;
//         }
//         catch (tf::TransformException &ex) {
//             ROS_ERROR("%s", ex.what());
//             ros::Duration(0.2).sleep();
//             continue;
//         }
//     }

//     ros::Subscriber sub = nh.subscribe("/ov_msckf/odomimu", 1, visionCallback);
//     ros::spin();

//     return 0;
// }
