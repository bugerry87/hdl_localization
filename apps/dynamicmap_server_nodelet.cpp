#include <set>
#include <sstream>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>

namespace hdl_localization {
class DynamicmapServerNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  DynamicmapServerNodelet() : map_frame("map") {}

  virtual ~DynamicmapServerNodelet() {}

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    map_frame = private_nh.param<std::string>("map_frame", "map");
    prefix = private_nh.param<std::string>("prefix", "maps/");

    dynamicmap_pub =
        nh.advertise<sensor_msgs::PointCloud2>("/dynamic_map", 1, true);
    sampledmap_pub =
        nh.advertise<sensor_msgs::PointCloud2>("/sampled_map", 1, true);
    query_sub =
        nh.subscribe("/initialpose", 1, &DynamicmapServerNodelet::query, this);
    auto dr = private_nh.param<double>("downsample_resolution", 0.0);

    dynamicmap.reset(new pcl::PointCloud<PointT>());
    sampledmap.reset(new pcl::PointCloud<PointT>());

    if (dr > 0.0) {
      voxelgrid.reset(new pcl::VoxelGrid<PointT>());
      voxelgrid->setLeafSize(dr, dr, dr);
      voxelgrid->setInputCloud(dynamicmap);
    }
  }

private:
  void query(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg) {
    std::stringstream ss;
    bool has_new = false;
    for (int xo = -150; xo <= 150; xo += 50) {
      for (int yo = -150; yo <= 150; yo += 50) {
        auto x = int(pose_msg->pose.pose.position.x + xo) / 100 * 100;
        auto y = int(pose_msg->pose.pose.position.y + yo) / 100 * 100;
        ss << prefix << "map_" << x << "_" << y << ".pcd";
        auto it = cached.find(ss.str());
        if (it == cached.end()) {
          cached.emplace(ss.str());
          pcl::PointCloud<PointT> submap;
          pcl::io::loadPCDFile(ss.str(), submap);
          dynamicmap->points.insert(dynamicmap->points.end(),
                                    submap.points.begin(), submap.points.end());
          has_new = true;
        }
        ss.str(std::string());
      }
    }

    if (has_new) {
      dynamicmap->header.frame_id = map_frame;
      dynamicmap->header.stamp = ros::Time::now().nsec;
      dynamicmap->header.seq += 1;
      dynamicmap_pub.publish(dynamicmap);

      if (voxelgrid.get()) {
        sampledmap->clear();
        voxelgrid->filter(*sampledmap);
        sampledmap->header = dynamicmap->header;
        sampledmap_pub.publish(sampledmap);
      }
    }
  }

  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Publisher dynamicmap_pub;
  ros::Publisher sampledmap_pub;
  ros::Subscriber query_sub;

  pcl::PointCloud<PointT>::Ptr dynamicmap;
  pcl::PointCloud<PointT>::Ptr sampledmap;
  boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid;
  std::string map_frame;
  std::string prefix;
  std::set<std::string> cached;
};

} // namespace hdl_localization

PLUGINLIB_EXPORT_CLASS(hdl_localization::DynamicmapServerNodelet,
                       nodelet::Nodelet)
