#include "custom_point_cloud_common.h"

namespace rviz_plugin_tutorials
{

CustomPointCloudCommon::CustomPointCloudCommon(rviz::Display* display) : rviz::PointCloudCommon(display)
{
}

// 在此处实现新的方法或重写基类方法
void CustomPointCloudCommon::update() {
//  if (!isEnabled()) {
//    return;
//  }

//  // 更新所有的子图
//  for (auto it = submap_cloud_map_.begin(); it != submap_cloud_map_.end(); ++it) {
//    const int submap_id = it->first;
//    const sensor_msgs::PointCloud2& submap_cloud = it->second;

//    // 获取子图在RViz中的名称
//    const std::string cloud_name = getCloudName(submap_id);

//    // 如果这个子图的点云已经在RViz中创建了，直接更新它
//    if (hasCloud(cloud_name)) {
//      updateCloud(cloud_name, submap_cloud);
//    } else {
//      // 如果这个子图的点云还没有在RViz中创建，则创建它并加入到显示中
//      addCloud(cloud_name, submap_cloud);
//    }
//  }
}

}  // namespace rviz_plugin_tutorials

