#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <localization/pose_estimator.hpp>


namespace localization {

class GlobalmapServerNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  GlobalmapServerNodelet() {
  }
  virtual ~GlobalmapServerNodelet() {
  }

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();  //Get the node handle with the Multi Threaded callback queue
    private_nh = getPrivateNodeHandle();

    initialize_params();

    // publish globalmap with "latched" publisher
    globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
    globalmap_pub.publish(globalmap);
  }

private:
  void initialize_params() {
    // read globalmap from a pcd file
    std::string globalmap_pcd = private_nh.param<std::string>("globalmap_pcd", "");
    //slam method 
    std::string slam_method = private_nh.param<std::string>("slam_method", "autoware");
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    globalmap->header.frame_id = "map";
    
    //pcl::PointCloud<PointT> cloud2;
    //转换loam生成pcd的角度 平行于栅格地图
    if(slam_method == "loam")
    {
      Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
      Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
      //绕x轴旋转一个theta角
      transform_2.rotate(Eigen::AngleAxisf(1.570795, Eigen::Vector3f::UnitX()));
      pcl::transformPointCloud(*globalmap, *globalmap, transform_2);
      //绕z轴旋转90度
      transform_1.rotate(Eigen::AngleAxisf(1.570795, Eigen::Vector3f::UnitZ()));
      pcl::transformPointCloud(*globalmap, *globalmap, transform_1); 
    }
    
    // downsample globalmap
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
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

}


PLUGINLIB_EXPORT_CLASS(localization::GlobalmapServerNodelet, nodelet::Nodelet)
