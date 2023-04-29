#include "auto_subscriber_display.h"
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <std_msgs/UInt32.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>
namespace rviz_plugin_tutorials
{

AutoSubscriberDisplay::AutoSubscriberDisplay() : submap_count_(0)
{
    ROS_INFO_STREAM("AutoSubscriberDisplay plugin loaded.");
    point_cloud_common_ = new rviz::PointCloudCommon(this);
}

AutoSubscriberDisplay::~AutoSubscriberDisplay()
{
}

void AutoSubscriberDisplay::onInitialize()
{
        ROS_INFO(__FUNCTION__);
  MessageFilterDisplay::onInitialize();
}

void AutoSubscriberDisplay::processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{

  processMessage(msg);
}
void AutoSubscriberDisplay::processMessage(const sensor_msgs::PointCloud2ConstPtr& msg)
{
//    sensor_msgs::PointCloud2 display_msg = *msg;

    // Call the method of the point_cloud_common_ object to actually display the point cloud
    point_cloud_common_->addMessage(msg);
}
void AutoSubscriberDisplay::update(float wall_dt, float ros_dt)
{
    ROS_INFO("AutoSubscriberDisplay::update");
  std_msgs::UInt32ConstPtr submap_count_msg =
      ros::topic::waitForMessage<std_msgs::UInt32>("submap_count", nh_, ros::Duration(0.1));

  if (submap_count_msg)
  {
    int new_submap_count = submap_count_msg->data;

    if (new_submap_count > submap_count_)
    {
      ROS_INFO("update submap count: %d", new_submap_count);
      for (int i = submap_count_; i < new_submap_count; ++i)
      {
        std::string topic = "submap_" + std::to_string(i) + "/points";
        ROS_INFO_STREAM("Subscribing to " << topic);
        subscribers_.push_back(nh_.subscribe<sensor_msgs::PointCloud2>(topic, 1, [this](const sensor_msgs::PointCloud2ConstPtr& msg) { processPointCloud(msg); }));
      }

      submap_count_ = new_submap_count;
    }
  }

  MessageFilterDisplay::update(wall_dt, ros_dt);
}

} // namespace rviz_plugin_tutorials

PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::AutoSubscriberDisplay, rviz::Display)
//PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::AutoSubscriberDisplay, rviz::MessageFilterDisplay<sensor_msgs::PointCloud2>)
