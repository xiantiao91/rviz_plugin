#include "auto_subscriber_display.h"
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <std_msgs/UInt32.h>
#include <pluginlib/class_list_macros.h>
//namespace rviz_plugin_tutorials
//{

AutoSubscriberDisplay::AutoSubscriberDisplay() : submap_count_(0)
{
    ROS_INFO_STREAM("AutoSubscriberDisplay plugin loaded.");
}

AutoSubscriberDisplay::~AutoSubscriberDisplay()
{
}

void AutoSubscriberDisplay::onInitialize()
{
        ROS_INFO(__FUNCTION__);
//  MessageFilterDisplay::onInitialize();
}
//void AutoSubscriberDisplay::processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
//{

////  processMessage(msg);
//}
void AutoSubscriberDisplay::processMessage(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // Do nothing here, because we will handle the message in processPointCloud()
}
//void AutoSubscriberDisplay::update(float wall_dt, float ros_dt)
//{
//  std_msgs::UInt32ConstPtr submap_count_msg =
//      ros::topic::waitForMessage<std_msgs::UInt32>("submap_count", nh_, ros::Duration(0.1));

//  if (submap_count_msg)
//  {
//    int new_submap_count = submap_count_msg->data;

//    if (new_submap_count > submap_count_)
//    {
//      for (int i = submap_count_; i < new_submap_count; ++i)
//      {
//        std::string topic = "submap_" + std::to_string(i) + "/points";
//        ROS_INFO_STREAM("Subscribing to " << topic);
////        subscribers_.push_back(nh_.subscribe(topic, 1, &AutoSubscriberDisplay::processPointCloud, this));
////        subscribers_.push_back(
////            nh_.subscribe<sensor_msgs::PointCloud2>(topic, 1, boost::bind(&AutoSubscriberDisplay::processPointCloud, this, _1, i)));
////        subscribers_.push_back(nh_.subscribe(topic, 1, boost::bind(&AutoSubscriberDisplay::processPointCloud, this, _1)));
//        subscribers_.push_back(nh_.subscribe<sensor_msgs::PointCloud2>(topic, 1, [this](const sensor_msgs::PointCloud2ConstPtr& msg) { processPointCloud(msg); }));


//      }

//      submap_count_ = new_submap_count;
//    }
//  }

////  MessageFilterDisplay::update(wall_dt, ros_dt);
//}

//} // namespace rviz_plugin_tutorials

PLUGINLIB_EXPORT_CLASS(AutoSubscriberDisplay, rviz::Display)
//PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::AutoSubscriberDisplay, rviz::MessageFilterDisplay<sensor_msgs::PointCloud2>)
