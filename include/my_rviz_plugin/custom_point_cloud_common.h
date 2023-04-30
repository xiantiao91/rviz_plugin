#pragma once

#include <rviz/default_plugin/point_cloud_common.h>
namespace rviz_plugin_tutorials
{

class CustomPointCloudCommon : public rviz::PointCloudCommon
{
public:
  CustomPointCloudCommon(rviz::Display* display);
  void addSubmap(int submap_id, const sensor_msgs::PointCloud2ConstPtr& msg) {
    submap_cloud_map_[submap_id] = *msg;
  }
//  void update(const std::string& topic, const sensor_msgs::PointCloud2ConstPtr& cloud_msg) override;
  std::map<int, sensor_msgs::PointCloud2> submap_cloud_map_;
  // 在此处添加新的方法或重写基类方法
  void update();

};

}  // namespace rviz_plugin_tutorials

