#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <thread>

ros::Publisher odom_pub[4];

tf::StampedTransform imu_cam_transform[4];
tf::Transform local_pose;

const int process_every_nth_frame_ = 2;
static int frame_counter_ = 0;
void visionCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    if (++frame_counter_ < process_every_nth_frame_) {
        return;
    }
    frame_counter_ = 0;
    geometry_msgs::Pose odom_pose = odom->pose.pose;
    tf::poseMsgToTF(odom_pose, local_pose);

    for (int i = 0; i < 4; i++) {
        geometry_msgs::PoseStamped pose_msg;
        tf::Transform global_cam_transform = local_pose*imu_cam_transform[i];
        pose_msg.header.stamp = odom->header.stamp;
        pose_msg.header.frame_id = "global";
        pose_msg.pose.position.x = global_cam_transform.getOrigin().x();
        pose_msg.pose.position.y = global_cam_transform.getOrigin().y();
        pose_msg.pose.position.z = global_cam_transform.getOrigin().z();
        pose_msg.pose.orientation.x = global_cam_transform.getRotation().x();
        pose_msg.pose.orientation.y = global_cam_transform.getRotation().y();
        pose_msg.pose.orientation.z = global_cam_transform.getRotation().z();
        pose_msg.pose.orientation.w = global_cam_transform.getRotation().w();

        odom_pub[i].publish(pose_msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_to_odom_publisher");
    ros::NodeHandle nh;

    odom_pub[0] = nh.advertise<geometry_msgs::PoseStamped>("/seeker/camera_pose", 10);
    odom_pub[1] = nh.advertise<geometry_msgs::PoseStamped>("odom_cam2", 10);
    odom_pub[2] = nh.advertise<geometry_msgs::PoseStamped>("odom_cam4", 10);
    odom_pub[3] = nh.advertise<geometry_msgs::PoseStamped>("odom_cam6", 10);
    tf::TransformListener listener;

    while (ros::ok()) {
        try {
            listener.lookupTransform("imu", "cam0", ros::Time(0), imu_cam_transform[0]);
            break;
        }
        catch (tf::TransformException &ex) {
            //ROS_ERROR("%s", ex.what());
            ros::Duration(0.2).sleep();
            continue;
        }
    }
    while (ros::ok()) {
        try {
            listener.lookupTransform("imu", "cam2", ros::Time(0), imu_cam_transform[1]);
            break;
        }
        catch (tf::TransformException &ex) {
            //ROS_ERROR("%s", ex.what());
            ros::Duration(0.2).sleep();
            continue;
        }
    }
    while (ros::ok()) {
        try {
            listener.lookupTransform("imu", "cam4", ros::Time(0), imu_cam_transform[2]);
            break;
        }
        catch (tf::TransformException &ex) {
            //ROS_ERROR("%s", ex.what());
            ros::Duration(0.2).sleep();
            continue;
        }
    }
    while (ros::ok()) {
        try {
            listener.lookupTransform("imu", "cam6", ros::Time(0), imu_cam_transform[3]);
            break;
        }
        catch (tf::TransformException &ex) {
            //ROS_ERROR("%s", ex.what());
            ros::Duration(0.2).sleep();
            continue;
        }
    }

    ros::Subscriber sub = nh.subscribe("/ov_msckf/odomimu", 1, visionCallback);
    ros::spin();

    return 0;
}
