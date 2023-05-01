#include "custom_point_cloud_common.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace rviz_plugin_tutorials
{

CustomPointCloudCommon::CustomPointCloudCommon(rviz::Display* display, rviz::DisplayContext *display_context) :
    rviz::PointCloudCommon(display),display_context_(display_context)
{
    custom_scene_node_ = display_context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
}


cache_point_type& CustomPointCloudCommon::getMapPointsById(int submap_idx)
{
    return map_points_[submap_idx];
}
void CustomPointCloudCommon::setPartialPointCloud(int submap_id, const sensor_msgs::PointCloud2 &cloud)
{
    // 将给定的局部点云存储在 map_points_ 中
    //    map_points_[submap_id] = *cloud;

    //    // 如果 submap_id 不存在于 map_renderers_ 中，则创建一个新的 Ogre::ManualObject 对象
    //    if (map_renderers_.find(submap_id) == map_renderers_.end())
    //    {
    //        Ogre::ManualObject* new_renderer = context_->getSceneManager()->createManualObject();
    //        new_renderer->setDynamic(true);
    //        map_renderers_[submap_id] = new_renderer;

    //        // 在此处设置新渲染器的属性，例如颜色、大小等
    //        // ...
    //    }

    //    // 更新特定子地图的点云渲染器
    //    Ogre::ManualObject* renderer = map_renderers_[submap_id];
    //    renderer->clear();

    //    // 设置新的点云数据
    //    // 这里需要将 sensor_msgs::PointCloud2 数据转换为 Ogre::Vector3 格式，并将其添加到渲染器中
    //    // ...

    //    // 更新渲染器的元数据，例如时间戳、frame_id 等
    //    // ...

    //    // 通知 rviz 进行重绘
    //    getDisplayContext()->queueRender();
    ////    context_->queueRender();
}

// 在此处实现新的方法或重写基类方法

void CustomPointCloudCommon::update(float wall_dt, float ros_dt)
{
//    ROS_INFO("CustomPointCloudCommon::update");

    mutex_map_points_.lock();
    for (auto& flag : update_flag_)
    {
        if (flag.second) // 如果需要更新
        {
            int submap_id = flag.first;
//            sensor_msgs::PointCloud2& cloud = map_points_[submap_id];

            // 更新子地图的点云渲染器
            auto it = point_cloud_renderers_.find(submap_id);

            // 如果找不到与子图 ID 关联的 rviz::PointCloud 对象，则创建一个新的 rviz::PointCloud 对象
            if (it == point_cloud_renderers_.end())
            {
                rviz::PointCloud *new_point_cloud = new rviz::PointCloud();
//                std::string point_name = ;
                new_point_cloud->setName("PointCloud" + std::to_string(submap_id));
                new_point_cloud->setRenderMode(rviz::PointCloud::RM_POINTS); // 设置渲染模式
                // 设置其他渲染属性，例如颜色、点大小等

                // 将新创建的 rviz::PointCloud 对象与子图 ID 关联起来
                point_cloud_renderers_[submap_id] = new_point_cloud;
                // 将新创建的 rviz::PointCloud 对象添加到 Ogre 场景管理器中
                custom_scene_node_->attachObject(new_point_cloud);
            }
            const cache_point_type& cache_points = map_points_[submap_id];
            pcl::PointCloud<pcl::PointXYZRGB> pcl_points;
            pcl::fromROSMsg(cache_points,pcl_points);
            std::vector<rviz::PointCloud::Point> rviz_point_coll;
            for (const pcl::PointXYZRGB& point : pcl_points.points) {
                rviz::PointCloud::Point rviz_point;
                rviz_point.position.x = point.x;
                rviz_point.position.y = point.y;
                rviz_point.position.z = point.z;
                rviz_point.color = Ogre::ColourValue(point.r / 255.0, point.g / 255.0, point.b / 255.0, 0.2); // 设置点的颜色（红色，不透明）
                rviz_point_coll.push_back(rviz_point);
                //                renderer->addPoints(&rviz_point, 1);
            }
            point_cloud_renderers_[submap_id]->clear();
            point_cloud_renderers_[submap_id]->addPoints(rviz_point_coll.data(),rviz_point_coll.size());
            //            rviz::PointCloud* renderer = point_cloud_renderers_[submap_id];
            //            renderer->clear();
            //            renderer->addPoints(cloud);
            //            mutex_map_points_.unlock();
            // 设置更新标志为 false
            flag.second = false;
        }
    }

    mutex_map_points_.unlock();
    // 更新 RViz 中的可视化效果
    display_context_->queueRender();
    // Call the base class update method
    rviz::PointCloudCommon::update(wall_dt, ros_dt);

    // Add your custom update logic for partial point clouds here
    // ...
}



//void CustomPointCloudCommon::update() {
////  if (!isEnabled()) {
////    return;
////  }

////  // 更新所有的子图
////  for (auto it = submap_cloud_map_.begin(); it != submap_cloud_map_.end(); ++it) {
////    const int submap_id = it->first;
////    const sensor_msgs::PointCloud2& submap_cloud = it->second;

////    // 获取子图在RViz中的名称
////    const std::string cloud_name = getCloudName(submap_id);

////    // 如果这个子图的点云已经在RViz中创建了，直接更新它
////    if (hasCloud(cloud_name)) {
////      updateCloud(cloud_name, submap_cloud);
////    } else {
////      // 如果这个子图的点云还没有在RViz中创建，则创建它并加入到显示中
////      addCloud(cloud_name, submap_cloud);
////    }
////  }
//}

}  // namespace rviz_plugin_tutorials

