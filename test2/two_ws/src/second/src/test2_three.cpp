#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_publisher");
    ros::NodeHandle nh;
    // 硬编码源点云和目标点云文件路径
    std::string source_pcd_file = "/home/dw/test2/source.pcd";
    std::string target_pcd_file = "/home/dw/test2/target.pcd";
    // 读取源点云和目标点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
    // 判断是否读取到了源点云数据
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(source_pcd_file, *cloud_source) ==
        -1)
    {
        ROS_ERROR("Couldn't read source file %s", source_pcd_file.c_str());
        return -1;
    }
    // 判断是否读取到了目标点云数据
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(target_pcd_file, *cloud_target) ==
        -1)
    {
        ROS_ERROR("Couldn't read target file %s", target_pcd_file.c_str());
        return -1;
    }
    // 执行点云配准算法
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; // 创建 ICP 对象
    // 设置点云
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    pcl::PointCloud<pcl::PointXYZ> cloud_matched; // 创建存储匹配后点云的对象
    icp.align(cloud_matched);                     // 执行 ICP 算法，进行点云配准
    if (icp.hasConverged())                       // 检查是否收敛
    {
        std::cout << "ICP converged with score: " << icp.getFitnessScore() << std::endl;
    }
    else
    {
        std::cerr << "ICP did not converge" << std::endl;
        return -1;
    }
    // 将pcl::PointCloud<pcl::PointXYZ>类型转换为sensor_msgs::PointCloud2类型
    // pcl主要用于点云处理和分析，另一个类型主要用于传输点云数据
    sensor_msgs::PointCloud2 cloud_msg;      // 创建 ROS 消息对象
    pcl::toROSMsg(cloud_matched, cloud_msg); // 转换为 ROS PointCloud2 消息
    cloud_msg.header.frame_id = "base_link"; // 假设匹配后的点云在base_link坐标系下
    // 创建一个发布器，发布PointCloud2类型的消息
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("matched_point_cloud", 1);
    // 循环发布点云数据
    ros::Rate loop_rate(10); // 设置发布频率为10Hz
    while (ros::ok())        // 当 ROS 节点正常运行时执行循环
    {
        cloud_msg.header.stamp = ros::Time::now(); // 更新消息的时间戳
        pub.publish(cloud_msg);                    // 发布点云消息
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
