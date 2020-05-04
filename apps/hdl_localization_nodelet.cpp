#include <boost/circular_buffer.hpp>
#include <iostream>
#include <mutex>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>

#include <hdl_localization/pose_estimator.hpp>

namespace hdl_localization {

class HdlLocalizationNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  HdlLocalizationNodelet() {}
  virtual ~HdlLocalizationNodelet() {}

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    processing_time.resize(16);
    initialize_params();

    map_frame_id = private_nh.param<std::string>("map_frame_id", "world");
    odom_child_frame_id =
        private_nh.param<std::string>("odom_child_frame_id", "base_link");

    use_guesses = private_nh.param<bool>("use_guesses", true);
    if (use_guesses) {
      NODELET_INFO("enable guess-based prediction");
      guess_sub = mt_nh.subscribe(
          "/guess", 256, &HdlLocalizationNodelet::guess_callback, this);
    }
    points_sub = mt_nh.subscribe(
        "/velodyne_points", 5, &HdlLocalizationNodelet::points_callback, this);
    globalmap_sub = nh.subscribe(
        "/globalmap", 1, &HdlLocalizationNodelet::globalmap_callback, this);
    initialpose_sub = nh.subscribe(
        "/initialpose", 8, &HdlLocalizationNodelet::initialpose_callback, this);

    pose_pub = nh.advertise<nav_msgs::Odometry>("/odom", 5, false);
    aligned_pub =
        nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 5, false);
  }

private:
  void initialize_params() {
    // intialize scan matching method
    double downsample_resolution =
        private_nh.param<double>("downsample_resolution", 0.1);
    std::string ndt_neighbor_search_method =
        private_nh.param<std::string>("ndt_neighbor_search_method", "DIRECT7");

    double ndt_resolution = private_nh.param<double>("ndt_resolution", 1.0);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(
        new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution,
                           downsample_resolution);
    downsample_filter = voxelgrid;

    pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(
        new pclomp::NormalDistributionsTransform<PointT, PointT>());
    ndt->setTransformationEpsilon(0.01);
    ndt->setResolution(ndt_resolution);
    if (ndt_neighbor_search_method == "DIRECT1") {
      NODELET_INFO("search_method DIRECT1 is selected");
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
    } else if (ndt_neighbor_search_method == "DIRECT7") {
      NODELET_INFO("search_method DIRECT7 is selected");
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    } else {
      if (ndt_neighbor_search_method == "KDTREE") {
        NODELET_INFO("search_method KDTREE is selected");
      } else {
        NODELET_WARN("invalid search method was given");
        NODELET_WARN("default method is selected (KDTREE)");
      }
      ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
    }
    registration = ndt;

    // initialize pose estimator
    if (private_nh.param<bool>("specify_init_pose", true)) {
      NODELET_INFO("initialize pose estimator with specified parameters!!");
      pose_estimator.reset(new hdl_localization::PoseEstimator(
          registration, ros::Time::now(),
          Eigen::Vector3f(private_nh.param<double>("init_pos_x", 0.0),
                          private_nh.param<double>("init_pos_y", 0.0),
                          private_nh.param<double>("init_pos_z", 0.0)),
          Eigen::Quaternionf(private_nh.param<double>("init_ori_w", 1.0),
                             private_nh.param<double>("init_ori_x", 0.0),
                             private_nh.param<double>("init_ori_y", 0.0),
                             private_nh.param<double>("init_ori_z", 0.0)),
          private_nh.param<double>("cool_time_duration", 0.5)));
    }
  }

private:
  /**
   * @brief callback for guesses
   * @param pose
   */
  void
  guess_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose) {
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    auto p = pose->pose.pose.position;
    auto o = pose->pose.pose.orientation;
    auto stamp = pose->header.stamp;
    Eigen::Vector3f pos(p.x, p.y, p.z);
    Eigen::Vector4f ori(o.w, o.x, o.y, o.z);
    pose_estimator->predict(stamp, pos, ori);
  }

  /**
   * @brief callback for point cloud data
   * @param points_msg
   */
  void points_callback(const sensor_msgs::PointCloud2ConstPtr &points_msg) {
    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
    if (!pose_estimator) {
      NODELET_ERROR("waiting for initial pose input!!");
      return;
    }

    if (!globalmap) {
      NODELET_ERROR("globalmap has not been received!!");
      return;
    }

    const auto &stamp = points_msg->header.stamp;
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *pcl_cloud);

    if (pcl_cloud->empty()) {
      NODELET_ERROR("cloud is empty!!");
      return;
    }

    // transform pointcloud into odom_child_frame_id
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    if (!tf_listener.waitForTransform(odom_child_frame_id, "velodyne", stamp,
                                      ros::Duration(0.5))) {
      NODELET_WARN("timeout at waiting for transformer update!!");
      return;
    }

    if (!pcl_ros::transformPointCloud(odom_child_frame_id, *pcl_cloud, *cloud,
                                      tf_listener)) {
      NODELET_WARN("point cloud cannot be transformed into target frame!!");
      return;
    }

    // predict
    auto q = pose_estimator->quat();
    Eigen::Vector4f ori(q.w(), q.x(), q.y(), q.z());
    pose_estimator->predict(stamp, pose_estimator->pos(), ori);

    // correct
    auto filtered = downsample(cloud);
    auto t1 = ros::WallTime::now();
    auto aligned = pose_estimator->correct(filtered);
    auto t2 = ros::WallTime::now();

    processing_time.push_back((t2 - t1).toSec());
    double avg_processing_time =
        std::accumulate(processing_time.begin(), processing_time.end(), 0.0) /
        processing_time.size();
    // NODELET_INFO_STREAM("processing_time: " << avg_processing_time * 1000.0
    // << "[msec]");

    if (aligned_pub.getNumSubscribers()) {
      aligned->header.frame_id = map_frame_id;
      aligned->header.stamp = cloud->header.stamp;
      aligned_pub.publish(aligned);
    }

    publish_odometry(points_msg->header.stamp, pose_estimator->matrix());
  }

  /**
   * @brief callback for globalmap input
   * @param points_msg
   */
  void globalmap_callback(const sensor_msgs::PointCloud2ConstPtr &points_msg) {
    NODELET_INFO("globalmap received!");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);
    globalmap = cloud;

    registration->setInputTarget(globalmap);
  }

  /**
   * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
   * @param pose_msg
   */
  void initialpose_callback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg) {
    NODELET_INFO("initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    const auto &p = pose_msg->pose.pose.position;
    const auto &q = pose_msg->pose.pose.orientation;
    pose_estimator.reset(new hdl_localization::PoseEstimator(
        registration, ros::Time::now(), Eigen::Vector3f(p.x, p.y, p.z),
        Eigen::Quaternionf(q.w, q.x, q.y, q.z),
        private_nh.param<double>("cool_time_duration", 0.5)));
  }

  /**
   * @brief downsampling
   * @param cloud   input cloud
   * @return downsampled cloud
   */
  pcl::PointCloud<PointT>::ConstPtr
  downsample(const pcl::PointCloud<PointT>::ConstPtr &cloud) const {
    if (!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const ros::Time &stamp, const Eigen::Matrix4f &pose) {
    // broadcast the transform over tf
    geometry_msgs::TransformStamped odom_trans =
        matrix2transform(stamp, pose, map_frame_id, odom_child_frame_id);
    pose_broadcaster.sendTransform(odom_trans);

    // publish the transform
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = map_frame_id;

    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    odom.child_frame_id = odom_child_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pose_pub.publish(odom);
  }

  /**
   * @brief convert a Eigen::Matrix to TransformedStamped
   * @param stamp           timestamp
   * @param pose            pose matrix
   * @param frame_id        frame_id
   * @param child_frame_id  child_frame_id
   * @return transform
   */
  geometry_msgs::TransformStamped
  matrix2transform(const ros::Time &stamp, const Eigen::Matrix4f &pose,
                   const std::string &frame_id,
                   const std::string &child_frame_id) {
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = pose(0, 3);
    odom_trans.transform.translation.y = pose(1, 3);
    odom_trans.transform.translation.z = pose(2, 3);
    odom_trans.transform.rotation = odom_quat;

    return odom_trans;
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  std::string map_frame_id;
  std::string odom_child_frame_id;

  bool use_guesses;
  ros::Subscriber guess_sub;
  ros::Subscriber points_sub;
  ros::Subscriber globalmap_sub;
  ros::Subscriber initialpose_sub;

  ros::Publisher pose_pub;
  ros::Publisher aligned_pub;
  tf::TransformBroadcaster pose_broadcaster;
  tf::TransformListener tf_listener;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;

  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;

  // processing time buffer
  boost::circular_buffer<double> processing_time;
};

} // namespace hdl_localization

PLUGINLIB_EXPORT_CLASS(hdl_localization::HdlLocalizationNodelet,
                       nodelet::Nodelet)
