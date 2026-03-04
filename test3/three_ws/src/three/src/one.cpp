#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <fstream>
#include <vector>
// 读取KITTI .bin格式点云
pcl::PointCloud<pcl::PointXYZI>::Ptr readKittiBin(const std::string& bin_path)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::fstream input(bin_path.c_str(), std::ios::in | std::ios::binary);
    if (!input.good())
    {
        ROS_ERROR("无法打开KITTI bin文件: %s", bin_path.c_str());
        return nullptr;
    }

    input.seekg(0, std::ios::end);
    size_t file_size = input.tellg();
    size_t point_num = file_size / (4 * sizeof(float)); // 修正：更严谨的计算（4个float类型）
    input.seekg(0, std::ios::beg);

    for (size_t i = 0; i < point_num; ++i)
    {
        pcl::PointXYZI point; // 改用pcl::PointXYZI类型
        float data[4];
        input.read((char*)&data, sizeof(float) * 4);
        point.x = data[0];
        point.y = data[1];
        point.z = data[2];
        point.intensity = data[3]; // 此时赋值intensity无报错，PointXYZI包含该成员
        cloud->push_back(point);
    }
    input.close();
    cloud->width = point_num;
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

int main(int argc, char** argv)
{
    // 1. 初始化ROS节点
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "kitti_ground_segmentation");
    ros::NodeHandle nh;

    ros::Publisher pub_ground = nh.advertise<sensor_msgs::PointCloud2>("/ground_point_cloud", 1);
    ros::Publisher pub_non_ground = nh.advertise<sensor_msgs::PointCloud2>("/non_ground_point_cloud", 1);
    std::string bin_path = "/mnt/hgfs/0000000000.bin";
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud = readKittiBin(bin_path);
    if (!raw_cloud || raw_cloud->empty())
    {
        ROS_ERROR("点云读取失败或点云为空！");
        return -1;
    }
    ROS_INFO("成功读取KITTI点云，共包含 %lu 个点", raw_cloud->size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*raw_cloud, *cloud_xyz); 
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> sac;
    sac.setOptimizeCoefficients(true); // 优化平面系数
    sac.setModelType(pcl::SACMODEL_PLANE); // 拟合平面模型
    sac.setMethodType(pcl::SAC_RANSAC); // 采用RANSAC鲁棒算法
    sac.setMaxIterations(1000); // 最大迭代次数
    sac.setDistanceThreshold(0.3); // 点到平面的距离阈值（地面点判定）
    sac.setInputCloud(cloud_xyz);
    sac.segment(*inliers, *coefficients);

    if (inliers->indices.empty())
    {
        ROS_ERROR("未检测到平面（地面）！");
        return -1;
    }
    ROS_INFO("检测到地面点 %lu 个，非地面点 %lu 个", 
             inliers->indices.size(), cloud_xyz->size() - inliers->indices.size());

    // 提取地面点云和非地面点云（保持XYZ类型，适配分割结果）
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_ground(new pcl::PointCloud<pcl::PointXYZ>);

    // 提取地面点
    extract.setInputCloud(cloud_xyz);
    extract.setIndices(inliers);
    extract.setNegative(false); // false=提取内点（地面点）
    extract.filter(*cloud_ground);
    // 提取非地面点
    extract.setNegative(true); // true=提取外点（非地面点）
    extract.filter(*cloud_non_ground);
    // 转换为ROS消息格式
    sensor_msgs::PointCloud2 msg_ground, msg_non_ground;
    pcl::toROSMsg(*cloud_ground, msg_ground);
    pcl::toROSMsg(*cloud_non_ground, msg_non_ground);
    // 设置消息坐标系（RViz需对应）
    msg_ground.header.frame_id = "velodyne";
    msg_non_ground.header.frame_id = "velodyne";
    ros::Rate loop_rate(1); // 1Hz频率发布
    // 循环发布点云（供RViz可视化）
    while (ros::ok())
    {
        msg_ground.header.stamp = ros::Time::now();
        msg_non_ground.header.stamp = ros::Time::now();
        pub_ground.publish(msg_ground);
        pub_non_ground.publish(msg_non_ground);

        ROS_INFO("已发布地面点云和非地面点云，可在RViz中查看");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}