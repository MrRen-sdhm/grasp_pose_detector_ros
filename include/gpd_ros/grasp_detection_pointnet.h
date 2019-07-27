

#ifndef GRASP_DETECTION_POINTNET_H_
#define GRASP_DETECTION_POINTNET_H_

// system
#include <algorithm>
#include <memory>
#include <vector>

// ROS
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// GPD
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector_pointnet.h>
#include <gpd/sequential_importance_sampling.h>

// this project (messages)
#include <gpd_ros/CloudIndexed.h>
#include <gpd_ros/CloudSamples.h>
#include <gpd_ros/CloudSources.h>
#include <gpd_ros/GraspConfig.h>
#include <gpd_ros/GraspConfigList.h>
#include <gpd_ros/SamplesMsg.h>

// this project (headers)
#include <gpd_ros/grasp_messages.h>
#include <gpd_ros/grasp_plotter.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;

/** GraspDetectionPointnet class
 *
 * \brief A ROS node that can detect grasp poses in a point cloud.
 *
 * This class is a ROS node that handles all the ROS topics.
 *
*/
class GraspDetectionPointnet
{
public:

  /**
   * \brief Constructor.
   * \param node the ROS node
  */
  GraspDetectionPointnet(ros::NodeHandle& node);

  /**
   * \brief Destructor.
  */
  ~GraspDetectionPointnet()
  {
    delete cloud_camera_;
//    delete importance_sampling_;
    delete grasp_detector_;
    delete rviz_plotter_;
  }

  /**
   * \brief Run the ROS node. Loops while waiting for incoming ROS messages.
  */
  void run();

  /**
   * \brief Detect grasp poses in a point cloud received from a ROS topic.
   * \return the list of grasp poses
  */
  std::vector<std::unique_ptr<gpd::candidate::Hand>> detectGraspPoses();


private:

  /**
   * \brief Find the indices of the points within a ball around a given point in the cloud.
   * \param cloud the point cloud
   * \param centroid the centroid of the ball
   * \param radius the radius of the ball
   * \return the indices of the points in the point cloud that lie within the ball
  */
  std::vector<int> getSamplesInBall(const PointCloudRGBA::Ptr& cloud, const pcl::PointXYZRGBA& centroid, float radius);

  /**
   * \brief Callback function for the ROS topic that contains the input point cloud.
   * \param msg the incoming ROS message
  */
  void cloud_callback(const sensor_msgs::PointCloud2& msg);

  /**
   * \brief Callback function for the ROS topic that contains the input point cloud and a list of indices.
   * \param msg the incoming ROS message
  */
  void cloud_indexed_callback(const gpd_ros::CloudIndexed& msg);

  /**
   * \brief Callback function for the ROS topic that contains the input point cloud and a list of (x,y,z) samples.
   * \param msg the incoming ROS message
  */
  void cloud_samples_callback(const gpd_ros::CloudSamples& msg);

  /**
   * \brief Initialize the <cloud_camera> object given a <cloud_sources> message.
   * \param msg the <cloud_sources> message
   */
  void initCloudCamera(const gpd_ros::CloudSources& msg);

  /**
   * \brief Callback function for the ROS topic that contains the input samples.
   * \param msg the incoming ROS message
  */
  void samples_callback(const gpd_ros::SamplesMsg& msg);

  Eigen::Matrix3Xd fillMatrixFromFile(const std::string& filename, int num_normals);

  Eigen::Vector3d view_point_; ///< (input) view point of the camera onto the point cloud

  gpd::util::Cloud* cloud_camera_; ///< stores point cloud with (optional) camera information and surface normals
  std_msgs::Header cloud_camera_header_; ///< stores header of the point cloud

  int size_left_cloud_; ///< (input) size of the left point cloud (when using two point clouds as input)
  bool has_cloud_, has_normals_, has_samples_; ///< status variables for received (input) messages
  std::string frame_; ///< point cloud frame
  ros::Subscriber cloud_sub_; ///< ROS subscriber for point cloud messages
  ros::Subscriber samples_sub_; ///< ROS subscriber for samples messages
  ros::Publisher grasps_pub_; ///< ROS publisher for grasp list messages
  ros::Publisher grasps_rviz_pub_; ///< ROS publisher for grasps in rviz (visualization)

  bool use_importance_sampling_; ///< if importance sampling is used
  bool use_rviz_; ///< if rviz is used for visualization instead of PCL
  std::vector<double> workspace_; ///< workspace limits

  gpd::GraspDetectorPointNet* grasp_detector_; ///< used to run the GPD algorithm
//gpd::SequentialImportanceSampling* importance_sampling_; ///< sequential importance sampling variation of GPD algorithm
  GraspPlotter* rviz_plotter_; ///< used to plot detected grasps in rviz

  /** constants for input point cloud types */
  static const int POINT_CLOUD_2; ///< sensor_msgs/PointCloud2
  static const int CLOUD_INDEXED; ///< gpd/CloudIndexed
  static const int CLOUD_SAMPLES; ///< gpd/CloudSamples
};

#endif /* GRASP_DETECTION_POINTNET_H_ */
