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

    initialize_params();

    // publish globalmap with "latched" publisher
    globalmap_pub =
        nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
    globalmap_pub.publish(globalmap);
  }

private:
  void initialize_params() {
    // read globalmap from a pcd file
    std::vector<std::string> globalmaps_pcd;
    private_nh.param("globalmaps_pcd", globalmaps_pcd, {""});
    globalmap.reset(new pcl::PointCloud<PointT>());
    globalmap->header.frame_id =
        private_nh.param<std::string>("map_frame", "map");
    globalmap->header.stamp = ros::Time::now().nsec;
    globalmap->header.seq = 1;

    for (auto submap_pcd : globalmaps_pcd) {
      pcl::PointCloud<PointT> submap;
      pcl::io::loadPCDFile(submap_pcd, submap);
      globalmap->points.insert(globalmap->points.end(), submap.points.begin(),
                               submap.points.end());
    }

    // downsample globalmap
    double downsample_resolution =
        private_nh.param<double>("downsample_resolution", 0.1);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(
        new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution,
                           downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Publisher globalmap_pub;

  pcl::PointCloud<PointT>::Ptr globalmap;
};

} // namespace hdl_localization

PLUGINLIB_EXPORT_CLASS(hdl_localization::GlobalmapServerNodelet,
                       nodelet::Nodelet)
