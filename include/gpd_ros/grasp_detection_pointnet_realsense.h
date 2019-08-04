#ifndef GRASP_DETECTION_POINTNET_REALSENSE_H_
#define GRASP_DETECTION_POINTNET_REALSENSE_H_

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
#include <gpd_ros/realsense2_viewer.h>

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
    void run(int loop_rate);

    /**
     * \brief Detect grasp poses in a point cloud received from a ROS topic.
     * \return the list of grasp poses
    */
    std::vector<std::unique_ptr<gpd::candidate::Hand>> detectGraspPoses();


private:

    /**
     * \brief Collect the input point cloud.
    */
    void collect_cloud();

    /**
     * \brief Callback function for the ROS topic that contains the input point cloud and a list of indices.
     * \param msg the incoming ROS message
    */
    void cloud_callback(const sensor_msgs::PointCloud2& msg);

    std::shared_ptr<RealsenseReceiver> receiver;

    Eigen::Vector3d view_point_; ///< (input) view point of the camera onto the point cloud

    gpd::util::Cloud* cloud_camera_; ///< stores point cloud with (optional) camera information and surface normals

    std::string frame_; ///< point cloud frame
    ros::Subscriber cloud_sub_; ///< ROS subscriber for point cloud messages

    ros::Publisher grasps_pub_; ///< ROS publisher for grasp list messages
    ros::Publisher grasps_rviz_pub_; ///< ROS publisher for grasps in rviz (visualization)

    bool use_rviz_; ///< if rviz is used for visualization instead of PCL

    gpd::GraspDetectorPointNet* grasp_detector_; ///< used to run the GPD algorithm
    GraspPlotter* rviz_plotter_; ///< used to plot detected grasps in rviz
};

#endif /* GRASP_DETECTION_POINTNET_REALSENSE_H_ */
