// #include <ros/ros.h>
// #include <rosbag/bag.h>
// #include <nav_msgs/Odometry.h>
// #include <mavros_msgs/PositionTarget.h>
// #include <boost/filesystem.hpp>
// #include <ctime>

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "record_node");
//     ros::NodeHandle nh;

//     // 获取当前日期
//     std::time_t t = std::time(nullptr);
//     char date[100];
//     std::strftime(date, sizeof(date), "%Y-%m-%d", std::localtime(&t));

//     // 创建bag文件路径
//     std::string package_path = ros::package::getPath("uav_server");
//     std::string data_dir = package_path + "/data";
//     boost::filesystem::create_directories(data_dir);
//     std::string bag_file = data_dir + "/uav_traj_" + date + ".bag";

//     // 打开bag文件
//     rosbag::Bag bag;
//     try {
//         bag.open(bag_file, rosbag::bagmode::Write);
//         ROS_INFO("Recording bag file: %s", bag_file.c_str());
//     } catch (const rosbag::BagException& e) {
//         ROS_ERROR("Failed to open bag file: %s", e.what());
//         return -1;
//     }

//     // 使用回调直接记录数据
//     auto odom_callback = [&](const nav_msgs::Odometry::ConstPtr& msg) {
//         bag.write("/uav_odom", ros::Time::now(), *msg);
//     };
//     auto target_callback = [&](const mavros_msgs::PositionTarget::ConstPtr& msg) {
//         bag.write("/mavros/setpoint_raw/local", ros::Time::now(), *msg);
//     };

//     // 订阅话题
//     ros::Subscriber odom_sub = nh.subscribe("/uav_odom", 1000, odom_callback);
//     ros::Subscriber target_sub = nh.subscribe("/mavros/setpoint_raw/local", 1000, target_callback);

//     // 保持运行
//     ros::spin();

//     // 关闭bag文件
//     bag.close();
//     ROS_INFO("Bag file saved to: %s", bag_file.c_str());
//     return 0;
// }
