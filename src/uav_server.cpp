#include <ros/ros.h>
#include <ros/package.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <fstream>
#include <sstream>
#include <nav_msgs/Odometry.h>

enum UAVState { IDLE, TAKEOFF, HOLD, EXECUTE_TRAJECTORY, LAND, ARM, DISARM }; 

std::string stateToString(UAVState state) {
    switch (state) {
        case IDLE: return "IDLE";
        case TAKEOFF: return "TAKEOFF";
        case HOLD: return "HOLD";
        case EXECUTE_TRAJECTORY: return "EXECUTE_TRAJECTORY";
        case LAND: return "LAND";
        case ARM: return "ARM";
        case DISARM: return "DISARM";
        default: return "UNKNOWN";
    }
}

UAVState uav_state = IDLE;
int previous_state = IDLE;  // 上一个状态
ros::Time state_entry_time; // 记录进入状态的时间
ros::Time last_odom_time;  // 用于记录最后一次接收到 odom 消息的时间
nav_msgs::Odometry uav_odom;
mavros_msgs::State current_state;
int rc_channel = 0;
size_t trajectory_index = 0;
bool odom_received = false; // 标记是否收到 odom 消息

void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void rcCallback(const mavros_msgs::RCInConstPtr& msg) {
    // 定义静态变量，用于记录是否已经进入过TAKEOFF状态
    static bool has_taken_off = false;

    // 获取第69号通道值
    uint16_t ch69_value = msg->channels[68]; 
    if (!has_taken_off && ch69_value > 1500) { 
        // 如果尚未进入过TAKEOFF状态且第69通道值大于1500
        uav_state = TAKEOFF; // 切换状态为 TAKEOFF
        has_taken_off = true; // 标记为已进入TAKEOFF状态
        return; // 在进入TAKEOFF后，立即返回，不再执行后续逻辑
    }

    // 获取第71号通道值
    uint16_t ch71_value = msg->channels[70]; 
    if (has_taken_off) { 
        // 仅在已进入过TAKEOFF状态后，响应71号通道的状态
        if (ch71_value < 500) {
            uav_state = EXECUTE_TRAJECTORY; // 切换状态为 EXECUTE_TRAJECTORY
        } else if (ch71_value > 1500) {
            uav_state = LAND; // 切换状态为 LAND
        } else {
            uav_state = HOLD; // 切换状态为 HOLD
        }
    }
}

bool readTrajectory(const std::string& file_path, std::vector<mavros_msgs::PositionTarget>& trajectory) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open trajectory file.");
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream stream(line);
        std::string value;
        mavros_msgs::PositionTarget point;
        // geometry_msgs::PoseStamped point;

        if (std::getline(stream, value, ',')) point.position.x = std::stod(value);
        if (std::getline(stream, value, ',')) point.position.y = std::stod(value);
        if (std::getline(stream, value, ',')) point.position.z = std::stod(value);

        trajectory.push_back(point);
    }

    file.close();
    return true;
}

void arm(ros::ServiceClient& arming_client, ros::ServiceClient& set_mode_client, ros::Publisher& local_pos_pub, double takeoff_height) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed.");
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled.");
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    uav_odom = *msg; // 更新全局变量
    odom_received = true; // 标记为已收到
    last_odom_time = ros::Time::now(); // 更新最后一次接收时间
    // ROS_INFO("Received odom: Position x=%.2f, y=%.2f, z=%.2f",
    //          uav_odom.pose.pose.position.x,
    //          uav_odom.pose.pose.position.y,
    //          uav_odom.pose.pose.position.z);

}


void disarm(ros::ServiceClient& arming_client, ros::Publisher& local_pos_pub) {

    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;
    if (arming_client.call(disarm_cmd) && disarm_cmd.response.success) {
        ROS_INFO("Vehicle disarmed.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "uav_server");
    ros::NodeHandle nh("~");

    // ROS parameters
    double loop_rate;
    double takeoff_height;
    std::string traj_file;
    nh.param("uav_server/traj_file", traj_file, std::string("/path/to/traj_0.txt"));
    nh.param("uav_server/loop_rate", loop_rate, 50.0);
    nh.param("uav_server/takeoff_height", takeoff_height, 1.0);

    // Subscribers and Publishers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, stateCallback);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 10, rcCallback);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/uav_odom", 10, odomCallback);

    // Service Clients
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // Main loop
    ros::Rate rate(loop_rate);
    std::vector<mavros_msgs::PositionTarget> trajectory;

    if (readTrajectory(traj_file, trajectory))
    {
        ROS_INFO("\033[1;32mSuccess read trajectory\033[0m");
    }

    while (ros::ok()) {

        ros::spinOnce();
        
        if (!odom_received || (ros::Time::now() - last_odom_time).toSec() > 2.0) {
            ROS_WARN_THROTTLE(2.0, "NO ODOM, CAN'T TAKE OFF.");
        }

        if (uav_state != previous_state) {
            ROS_INFO("\033[1;33mSwitch state %s -> %s\033[0m", 
                    stateToString(static_cast<UAVState>(previous_state)).c_str(), 
                    stateToString(static_cast<UAVState>(uav_state)).c_str());
            previous_state = uav_state;
            state_entry_time = ros::Time::now(); // 更新状态进入时间
        }


        // //the uav_pose must satisfy the quadrotor frame. 
        //         x = y
        //         y = x
        //         z = -z
        switch (uav_state) 
        {
            case IDLE:
                break;

            case ARM:
                // arm(arming_client, set_mode_client, local_pos_pub, takeoff_height);
                uav_state = TAKEOFF;
                break;

            case TAKEOFF: {
                mavros_msgs::PositionTarget takeoff_pose;
                takeoff_pose.header.stamp = ros::Time::now();     // 当前时间
                takeoff_pose.header.frame_id = "world";           // 坐标系设置为 world
                takeoff_pose.position.x = 0.0;
                takeoff_pose.position.y = 0.0;
                takeoff_pose.position.z =-takeoff_height;
                // takeoff_pose.pose.position.x = 0.0;
                // takeoff_pose.pose.position.y = 0.0;
                // takeoff_pose.pose.position.z = -takeoff_height;
                local_pos_pub.publish(takeoff_pose);

                if (uav_odom.pose.pose.position.z >= takeoff_pose.position.z) {
                    uav_state = HOLD;
                }
                break;
            }

            case HOLD: {
                // mavros_msgs::PositionTarget hold_pose;
                // hold_pose.position.x = uav_odom.pose.pose.position.y;
                // hold_pose.position.y = uav_odom.pose.pose.position.x;
                // hold_pose.position.z = -uav_odom.pose.pose.position.z;
                // local_pos_pub.publish(hold_pose); 
                break;
            }

            case EXECUTE_TRAJECTORY:
                if (trajectory_index < trajectory.size()) {
                     mavros_msgs::PositionTarget traj_point;
                     traj_point.header.stamp = ros::Time::now();     // 当前时间
                     traj_point.header.frame_id = "world";           // 坐标系设置为 world
                     traj_point.position.x =  trajectory[trajectory_index].position.y;
                     traj_point.position.y =  trajectory[trajectory_index].position.x;
                     traj_point.position.z =  -trajectory[trajectory_index].position.z;
                    local_pos_pub.publish(traj_point);
                    trajectory_index++;
                } else {
                    // trajectory_index = 0; 
                    uav_state = HOLD;     
                }
                break;

            case LAND: {
                static bool land_pose_published = false;
                if (!land_pose_published) {
                    mavros_msgs::PositionTarget land_pose;
                    land_pose.header.stamp = ros::Time::now();     // 当前时间
                    land_pose.header.frame_id = "world";           // 坐标系设置为 world
                    land_pose.position.x = uav_odom.pose.pose.position.y;
                    land_pose.position.y = uav_odom.pose.pose.position.x;
                    land_pose.position.z = 0.0;
                    local_pos_pub.publish(land_pose);

                    land_pose_published = true; // 确保只发布一次
                }

                if ((ros::Time::now() - state_entry_time).toSec() >= 6.0) {
                    uav_state = DISARM;
                }
                break;
            }

            case DISARM:
                disarm(arming_client, local_pos_pub);
                uav_state = IDLE;
                break;
        }


        rate.sleep();
    }

    return 0;
}
