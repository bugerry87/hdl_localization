#include <vector>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>

namespace hdl_localization {
class GlobalmapServerNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  GlobalmapServerNodelet() {}

  virtual ~GlobalmapServerNodelet() {}

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    // read globalmap from a pcd file
    std::vector<std::string> globalmaps_pcd;
    private_nh.param("globalmaps_pcd", globalmaps_pcd, {""});
    pcl::PointCloud<PointT>::Ptr globalmap(new pcl::PointCloud<PointT>());
    globalmap->header.frame_id =
        private_nh.param<std::string>("map_frame", "map");
    globalmap->header.stamp = ros::Time::now().nsec;
    globalmap->header.seq = 1;

    globalmap_pub =
        nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
    sampledmap_pub =
        nh.advertise<sensor_msgs::PointCloud2>("/sampled_map", 5, true);

    for (auto submap_pcd : globalmaps_pcd) {
      pcl::PointCloud<PointT> submap;
      pcl::io::loadPCDFile(submap_pcd, submap);
      globalmap->points.insert(globalmap->points.end(), submap.points.begin(),
                               submap.points.end());
    }
    globalmap_pub.publish(globalmap);

    // downsample globalmap
    double downsample_resolution =
        private_nh.param<double>("downsample_resolution", 0.0);
    if (downsample_resolution > 0.0) {
      boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(
          new pcl::VoxelGrid<PointT>());
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution,
                             downsample_resolution);
      voxelgrid->setInputCloud(globalmap);

      pcl::PointCloud<PointT>::Ptr sampledmap(new pcl::PointCloud<PointT>());
      voxelgrid->filter(*sampledmap);
      sampledmap_pub.publish(sampledmap);
    }
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Publisher globalmap_pub;
  ros::Publisher sampledmap_pub;
};

} // namespace hdl_localization

PLUGINLIB_EXPORT_CLASS(hdl_localization::GlobalmapServerNodelet,
                       nodelet::Nodelet)
