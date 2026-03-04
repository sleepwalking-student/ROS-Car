# ROS智能驾驶小车
本实验课程设计，基于ROS系统与Autoware平台，开发集成激光雷达感知、点云处理、自主导航功能的机器人系统，涵盖语义地图标注、点云分割聚类、NDT/ICP 配准及路径规划，实现机器人在指定场景下的精准定位。
# test2
通过课程提供的车辆程序采集到真实场景的离线数据包点云数据，将其中通过开源算法Aloam进行建图，同时可以通过RVIZ查看建图过程。
<img width="1000" height="500" alt="image" src="https://github.com/user-attachments/assets/ddc67157-c989-460d-9756-8e12467e9503" />
<img width="1000" height="500" alt="image" src="https://github.com/user-attachments/assets/082f7bf4-a00b-4336-adb9-4660506484dd" />
# test3
启动激光雷达，对激光雷达扫描的障碍物物体进行分类聚类，并且使用rviz实时显示聚类中心和到激光雷达的中心距离以及显示聚类边框。
<img width="280" height="340" alt="image" src="https://github.com/user-attachments/assets/e3c6087c-8365-417c-84c8-dfc145029dbf" />
# testCSV
通过Unity工具，在.pcd文件中绘制行驶车道线和路沿线，并对车道线和路沿线进行离散操作。注意车道线和路沿线都需要紧贴着地面，不可悬空，不然自主导航时车辆会识别不到行驶车道线，最后将语义地图(.csv)文件导出并放入车辆工控机中进行验证是否能够实现自主导航。
<img width="692" height="232" alt="image" src="https://github.com/user-attachments/assets/ab76a403-13cb-4788-a4ed-5301bc25ca89" />
