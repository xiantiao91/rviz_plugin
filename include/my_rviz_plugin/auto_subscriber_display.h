#pragma once
#include <rviz/display.h>
#include <rviz/message_filter_display.h>
#include <sensor_msgs/PointCloud2.h>
#include <rviz/default_plugin/point_cloud_common.h>
//#include <rviz

namespace rviz_plugin_tutorials
{

class AutoSubscriberDisplay : public rviz::/*Display*/MessageFilterDisplay<sensor_msgs::PointCloud2>
{
   Q_OBJECT
public:
   AutoSubscriberDisplay();
  virtual ~AutoSubscriberDisplay();


protected:
  virtual void onInitialize();
  virtual void update(float wall_dt, float ros_dt);
  virtual void processMessage(const sensor_msgs::PointCloud2ConstPtr& msg);

private:
  ros::NodeHandle nh_;
  int submap_count_;
  std::vector<ros::Subscriber> subscribers_;
  void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);private:
  rviz::PointCloudCommon* point_cloud_common_;
};

}
