#pragma once
#include <rviz/display.h>
#include <rviz/message_filter_display.h>
#include <sensor_msgs/PointCloud2.h>

namespace rviz_plugin_tutorials
{

class AutoSubscriberDisplay : public rviz::MessageFilterDisplay<sensor_msgs::PointCloud2>
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
  void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);
};

}
