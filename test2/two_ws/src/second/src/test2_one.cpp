/*使用ICP算法计算出source.pcd和target.pcd的旋转矩阵R和平移矩阵t，并在终端输出。
  终端输出source点云和target点云的匹配分数score。*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
#include <Eigen/Dense>
int main (int argc, char** argv)
{
  
  // 定义点云指针
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned (new pcl::PointCloud<pcl::PointXYZ>);

  // 加载点云数据   
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/dw/test2/source.pcd", *source) == -1)
  {
    PCL_ERROR ("Couldn't read source file\n");
    return (-1);
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/dw/test2/target.pcd", *target) == -1)
  {
    PCL_ERROR ("Couldn't read target file\n");
    return (-1);
  }

  // 初始化ICP算法
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);

  // 执行配准
  icp.align(*aligned);

  // 输出结果
  if (icp.hasConverged())
  {
    std::cout << "ICP has converged, score is " << icp.getFitnessScore() << std::endl;
    std::cout << "Transformation matrix:" << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    // 提取旋转矩阵 R 和平移向量 t
    Eigen::Matrix4f transform = icp.getFinalTransformation();
    Eigen::Matrix3f R = transform.block<3,3>(0,0);
    Eigen::Vector3f t = transform.block<3,1>(0,3);
    std::cout << "Rotation matrix R:\n" << R << std::endl;
    std::cout << "Translation vector t:\n" << t.transpose() << std::endl;
  }
  else
  {
    std::cout << "ICP did not converge." << std::endl;
    return (-1);
  }

  return (0);
}