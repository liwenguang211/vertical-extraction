#include <Eigen/Geometry>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>

#include "include/STDesc.h"
#include "ros/init.h"

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

pcl::PointCloud<pcl::PointXYZI> clouds;
std::mutex laser_mtx;
std::mutex odom_mtx;
bool laserInsert = false;
int laser_cnt = 0;
std::queue<sensor_msgs::PointCloud2::ConstPtr> laser_buffer;
std::queue<nav_msgs::Odometry::ConstPtr> odom_buffer;

void laserCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  std::unique_lock<std::mutex> lock(laser_mtx);

  laser_buffer.push(msg);
  std::cout << "recv liDAR data width=" << msg->width << "  height=" << msg->height << std::endl;
  std::cout << "rlaser_buffer size=" << laser_buffer.size() << std::endl;
  laser_cnt++;
  // if (laser_cnt < 100)
  // laser_buffer.pop();
  // if (laser_buffer.size() >= 20 && (!laserInsert)) {
  //   laserInsert = true;

  //   for (std::size_t i = 0; i < 20; i++) {
  //     auto laser_msg = laser_buffer.front();
  //     pcl::PointCloud<pcl::PointXYZI> lidar_data;
  //     pcl::fromROSMsg(*laser_msg, lidar_data);
  //     clouds = clouds+lidar_data;
  //     laser_buffer.pop();
  //   }
  //   pcl::io::savePCDFileBinary("/home/shinva/mappingData/map_combined.pcd", clouds);
  //   std::cout << "20 frame total points size=" << clouds.size() << "  laser cnt  " << laser_cnt <<std::endl;
  // }
}

void OdomHandler(const nav_msgs::Odometry::ConstPtr &msg) {
  std::unique_lock<std::mutex> lock(odom_mtx);

  odom_buffer.push(msg);
}

bool syncPackages(PointCloud::Ptr &cloud, Eigen::Affine3d &pose) {
  if (laser_buffer.empty() || odom_buffer.empty())
    return false;

  auto laser_msg = laser_buffer.front();
  double laser_timestamp = laser_msg->header.stamp.toSec();

  auto odom_msg = odom_buffer.front();
  double odom_timestamp = odom_msg->header.stamp.toSec();

  // check if timestamps are matched
  if (abs(odom_timestamp - laser_timestamp) < 1e-3) {
    pcl::fromROSMsg(*laser_msg, *cloud);

    Eigen::Quaterniond r(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    Eigen::Vector3d t(odom_msg->pose.pose.position.x,
                      odom_msg->pose.pose.position.y,
                      odom_msg->pose.pose.position.z);

    pose = Eigen::Affine3d::Identity();
    pose.translate(t);
    pose.rotate(r);

    std::unique_lock<std::mutex> l_lock(laser_mtx);
    std::unique_lock<std::mutex> o_lock(odom_mtx);

    laser_buffer.pop();
    odom_buffer.pop();

  } else if (odom_timestamp < laser_timestamp) {
    ROS_WARN("Current odometry is earlier than laser scan, discard one "
             "odometry data.");
    std::unique_lock<std::mutex> o_lock(odom_mtx);
    odom_buffer.pop();
    return false;
  } else {
    ROS_WARN(
        "Current laser scan is earlier than odometry, discard one laser scan.");
    std::unique_lock<std::mutex> l_lock(laser_mtx);
    laser_buffer.pop();
    return false;
  }

  return true;
}

void update_poses(const gtsam::Values &estimates,
                  std::vector<Eigen::Affine3d> &poses) {
  assert(estimates.size() == poses.size());

  poses.clear();

  for (int i = 0; i < estimates.size(); ++i) {
    auto est = estimates.at<gtsam::Pose3>(i);
    Eigen::Affine3d est_affine3d(est.matrix());
    poses.push_back(est_affine3d);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "online_demo");
  ros::NodeHandle nh;

  std::string result_path = "";
  std::string pointCloudsPath = "";
  nh.param<std::string>("result_path", result_path, "");
  nh.param<std::string>("pointCloudsPath", pointCloudsPath, "");

  ConfigSetting config_setting;
  read_parameters(nh, config_setting);

  // ros::Publisher pubOdomAftMapped =
  // nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  ros::Publisher pubCurrentCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
  ros::Publisher pubCurrentCorner =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
  ros::Publisher pubMatchedCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
  ros::Publisher pubMatchedCorner =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
  ros::Publisher pubSTD =
      nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);

  ros::Publisher pubOriginCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_origin", 10000);

  ros::Publisher pubCorrectCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_correct", 10000);
  ros::Publisher pubCorrectPath =
      nh.advertise<nav_msgs::Path>("/correct_path", 100000);

  ros::Publisher pubOdomOrigin =
      nh.advertise<nav_msgs::Odometry>("/odom_origin", 10);
  ros::Publisher pubLoopConstraintEdge =
      nh.advertise<visualization_msgs::MarkerArray>("/loop_closure_constraints",
                                                    10);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "/velodyne_points", 100, laserCloudHandler);
  ros::Subscriber subOdom =
      nh.subscribe<nav_msgs::Odometry>("/Odometry", 100, OdomHandler);

  std::cout << "begin initiall ~~~~~~~~~~~~~~~~~~~~~: " << std::endl;
  STDescManager *std_manager = new STDescManager(config_setting);

  gtsam::Values initial;
  gtsam::NonlinearFactorGraph graph;

  // https://github.com/TixiaoShan/LIO-SAM/blob/6665aa0a4fcb5a9bb3af7d3923ae4a035b489d47/src/mapOptmization.cpp#L1385
  gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
      gtsam::noiseModel::Diagonal::Variances(
          (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

  gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
      gtsam::noiseModel::Diagonal::Variances(
          (gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8)
              .finished()); // rad*rad, meter*meter

  double loopNoiseScore = 1e-1;
  gtsam::Vector robustNoiseVector6(
      6); // gtsam::Pose3 factor has 6 elements (6D)
  robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore,
      loopNoiseScore, loopNoiseScore, loopNoiseScore;
  gtsam::noiseModel::Base::shared_ptr robustLoopNoise =
      gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Cauchy::Create(1),
          gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  gtsam::ISAM2 isam(parameters);

  size_t cloudInd = 0;
  size_t keyCloudInd = 0;

  std::vector<PointCloud::Ptr> cloud_vec;
  std::vector<Eigen::Affine3d> pose_vec;
  std::vector<Eigen::Affine3d> origin_pose_vec;
  std::vector<Eigen::Affine3d> key_pose_vec;
  std::vector<std::pair<int, int>> loop_container;

  PointCloud::Ptr key_cloud(new PointCloud);

  bool has_loop_flag = false;
  gtsam::Values curr_estimate;
  ros::Rate loop(500);
  ros::Rate slow_loop(10);
  Eigen::Affine3d last_pose;
  last_pose.setIdentity();
  std::cout << "Sucessfully initialized ~~~~~~~~~~~~~~~~~~~~~: " << std::endl;

  pcl::io::loadPCDFile<pcl::PointXYZI>("/home/shinva/mappingData/GlobalMap7.0.pcd", clouds);
  //pcl::io::loadPCDFile<pcl::PointXYZI>("/home/shinva/mappingData/kitti027.pcd", clouds);

  //pcl::io::loadPCDFile<pcl::PointXYZI>("/home/shinva/mappingData/jsr_GlobalMap7.0.pcd", clouds);
  std::cout << "loaded points size=" << clouds.size() <<std::endl;


  auto t_map_update_begin = std::chrono::high_resolution_clock::now();  
  // pcl::io::savePCDFileBinary("/home/shinva/mappingData/tree0.pcd", *current_cloud);
            
  down_sampling_voxel_png(clouds, config_setting.ds_size_,
    config_setting.effective_count,
    config_setting.pole_count,
    config_setting.surround_count,
    config_setting.stand_out_count, false, 3, result_path);  // campus 7.0(0.9, 9 , 3, 3, 5,false, 4)) map dowm sample

  //down_sampling_voxel_png(clouds, 0.4, 2  , 6, 4, 12,false, 3);  // kitti027 dowm sample
  

  // down_sampling_voxel_png(clouds, 0.4, 2 , 7, 4, 15,false, 3);  // jingshi road dowm sample (clouds, 0.4, 2 , 5, 4, 4,false, 3)
  //down_sampling_voxel_png(*current_cloud, 0.3, 3 , 1, 40,true, 4);  // tree0 map dowm sample
  auto t_map_update_end = std::chrono::high_resolution_clock::now();
  double deltatime = time_inc(t_map_update_end,t_map_update_begin);
  std::cout<< "delta time = "<< deltatime <<std::endl;

  while (ros::ok()) {
    ros::spinOnce();
    slow_loop.sleep();
    continue;
    
      //   PointCloud correct_cloud;
      //   pcl::transformPointCloud(*current_cloud_body, correct_cloud,
      //                            latest_estimate_affine3d);
      //   sensor_msgs::PointCloud2 pub_cloud;
      //   pcl::toROSMsg(correct_cloud, pub_cloud);
      //   pub_cloud.header.frame_id = "camera_init";
      //   pubCorrectCloud.publish(pub_cloud);

      //visualizeLoopClosure(pubLoopConstraintEdge, loop_container, pose_vec);

      has_loop_flag = false;
      ++cloudInd;

  }

  // You can save full map with refined pose
  // assert(cloud_vec.size() == pose_vec.size());
  // PointCloud full_map;
  // for (int i = 0; i < pose_vec.size(); ++i) {
  //     PointCloud correct_cloud;
  //     pcl::transformPointCloud(*cloud_vec[i], correct_cloud, pose_vec[i]);
  //     full_map += correct_cloud;
  // }
  // down_sampling_voxel(full_map, 0.05);

  // std::cout << "saving map..." << std::endl;
  // pcl::io::savePCDFileBinary("/home/dustier/data/map.pcd", full_map);

  return 0;
}

