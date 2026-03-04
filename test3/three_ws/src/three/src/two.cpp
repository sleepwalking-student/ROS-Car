#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
// 修正1：补充缺失的头文件（包含compute3DCentroid和getMinMax3D）
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

// 全局发布器（用于可视化聚类中心和边框）
ros::Publisher pub_cluster_centers;
ros::Publisher pub_cluster_boxes;

// 点云回调函数：处理实时点云、聚类、发布可视化信息
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_cloud)
{
    // 1. ROS点云转换为PCL点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*ros_cloud, *raw_cloud);

    // 2. 体素滤波降采样（减少计算量，提高实时性）
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setInputCloud(raw_cloud);
    voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f); // 10cm体素大小
    voxel_grid.filter(*filtered_cloud);

    // 3. 地面分割（先去除地面，只聚类障碍物）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*filtered_cloud, *cloud_xyz);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> sac;
    sac.setOptimizeCoefficients(true);
    sac.setModelType(pcl::SACMODEL_PLANE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setDistanceThreshold(0.2);
    sac.setInputCloud(cloud_xyz);
    sac.segment(*ground_inliers, *coefficients);

    // 提取非地面点（障碍物点云）
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(filtered_cloud);
    extract.setIndices(ground_inliers);
    extract.setNegative(true); // 提取非地面点
    extract.filter(*obstacle_cloud);

    if (obstacle_cloud->empty()) return;

    // 4. Euclidean聚类（分割不同障碍物）
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    kdtree->setInputCloud(obstacle_cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.5); // 聚类距离阈值（50cm内视为同一物体）
    ec.setMinClusterSize(20);    // 最小聚类点数（过滤噪声）
    ec.setMaxClusterSize(2000);  // 最大聚类点数（避免超大聚类）
    ec.setSearchMethod(kdtree);
    ec.setInputCloud(obstacle_cloud);
    ec.extract(cluster_indices);

    // 5. 构建可视化消息（聚类中心+边框）
    visualization_msgs::MarkerArray center_markers;
    visualization_msgs::MarkerArray box_markers;
    int cluster_id = 0;

    for (const auto& indices : cluster_indices)
    {
        // 提取单个聚类点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (int idx : indices.indices)
            cluster_cloud->push_back((*obstacle_cloud)[idx]);

        // 计算聚类中心
        Eigen::Vector4f centroid;
        // 修正2：函数调用正常（头文件包含后可识别）
        pcl::compute3DCentroid(*cluster_cloud, centroid);
        double distance = sqrt(centroid[0]*centroid[0] + centroid[1]*centroid[1] + centroid[2]*centroid[2]);

        // 构建聚类中心标记（文本+点）
        visualization_msgs::Marker center_marker;
        center_marker.header = ros_cloud->header;
        center_marker.ns = "cluster_centers";
        center_marker.id = cluster_id;
        center_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        center_marker.action = visualization_msgs::Marker::ADD;
        center_marker.pose.position.x = centroid[0];
        center_marker.pose.position.y = centroid[1];
        center_marker.pose.position.z = centroid[2] + 0.5; // 文本悬浮在聚类上方
        center_marker.text = "Dist: " + std::to_string(distance).substr(0, 5) + "m";
        center_marker.scale.z = 0.3; // 文本大小
        center_marker.color.r = 1.0;
        center_marker.color.g = 0.0;
        center_marker.color.b = 0.0;
        center_marker.color.a = 1.0;
        center_marker.lifetime = ros::Duration(0.1); // 实时更新（短生命周期）
        center_markers.markers.push_back(center_marker);

        // 计算聚类包围盒
        Eigen::Vector4f min_pt, max_pt;
        // 修正3：适配函数重载，传入正确的点云指针（直接解引用，匹配参数要求）
        pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);

        // 构建聚类边框标记
        visualization_msgs::Marker box_marker;
        box_marker.header = ros_cloud->header;
        box_marker.ns = "cluster_boxes";
        box_marker.id = cluster_id;
        box_marker.type = visualization_msgs::Marker::CUBE;
        box_marker.action = visualization_msgs::Marker::ADD;
        box_marker.pose.position.x = (min_pt[0] + max_pt[0]) / 2;
        box_marker.pose.position.y = (min_pt[1] + max_pt[1]) / 2;
        box_marker.pose.position.z = (min_pt[2] + max_pt[2]) / 2;
        box_marker.scale.x = max_pt[0] - min_pt[0];
        box_marker.scale.y = max_pt[1] - min_pt[1];
        box_marker.scale.z = max_pt[2] - min_pt[2];
        box_marker.color.r = 0.0;
        box_marker.color.g = 1.0;
        box_marker.color.b = 0.0;
        box_marker.color.a = 0.5; // 半透明
        box_marker.lifetime = ros::Duration(0.1);
        box_markers.markers.push_back(box_marker);

        cluster_id++;
    }

    // 6. 发布可视化消息（供RViz显示）
    pub_cluster_centers.publish(center_markers);
    pub_cluster_boxes.publish(box_markers);
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "lidar_obstacle_clustering");
    ros::NodeHandle nh;

    // 订阅激光雷达点云话题（需匹配bag文件中的点云话题）
    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/points_raw", 1, cloudCallback);

    // 初始化可视化发布器
    pub_cluster_centers = nh.advertise<visualization_msgs::MarkerArray>("/cluster_centers", 1);
    pub_cluster_boxes = nh.advertise<visualization_msgs::MarkerArray>("/cluster_boxes", 1);

    ROS_INFO("激光雷达障碍物聚类节点已启动，可在RViz中查看结果");

    // 循环等待回调
    ros::spin();
    return 0;
}