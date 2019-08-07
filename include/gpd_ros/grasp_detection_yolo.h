#ifndef GRASP_DETECTION_YOLO_
#define GRASP_DETECTION_YOLO_

// system
#include <algorithm>
#include <memory>
#include <vector>
#include <stdlib.h>

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

// Opencv
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

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

#define DEBUG 0

class YoloDetector {
public:
    YoloDetector(std::string config_file, std::string weights_file);

    // Get the bounding boxes of objects
    std::vector<cv::RotatedRect> getObjRect(cv::Mat image);

private:
    // Remove the bounding boxes with low confidence using non-maxima suppression
    std::vector<cv::RotatedRect> postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs, float confThreshold,
                                      float nmsThreshold, std::vector<std::string> classes);

    // Draw the predicted bounding box
    void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame,
                  std::vector<std::string> classes);

    // Get the names of the output layers
    std::vector<cv::String> getOutputsNames(const cv::dnn::Net& net);


    // yolo相关参数
    cv::dnn::Net net_;
    float confThreshold_ = 0.75; // Confidence threshold
    float nmsThreshold_ = 0.4;  // Non-maximum suppression threshold
    int inpWidth_ = 416;  // Width of network's input image 416 608
    int inpHeight_ = 416; // Height of network's input image 416 608
    static const int classes_num = 4;
    std::string classes_[classes_num]={"cube", "cubiod", "hexagonal", "triangular"};

    std::vector<std::string> get_classes_vec() {
        std::vector<std::string> classes_vec(classes_, classes_ + classes_num);
        return classes_vec;
    }
};


typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;

/** GraspDetectionYolo class
 *
 * \brief A ROS node that can detect grasp poses in a point cloud.
 *
 * This class is a ROS node that handles all the ROS topics.
 *
*/
class GraspDetectionYolo
{
public:

    /**
     * \brief Constructor.
     * \param node the ROS node
    */
    GraspDetectionYolo(ros::NodeHandle& node);

    /**
     * \brief Destructor.
    */
    ~GraspDetectionYolo()
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
    void collect_image_cloud();

    /**
     * \brief Callback function for the ROS topic that contains the input point cloud and a list of indices.
     * \param msg the incoming ROS message
    */
    void cloud_callback(const sensor_msgs::PointCloud2& msg);

    std::shared_ptr<RealsenseReceiver> receiver;

    Eigen::Vector3d view_point_; ///< (input) view point of the camera onto the point cloud

    cv::Mat image_;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;

    gpd::util::Cloud* cloud_camera_; ///< stores point cloud with (optional) camera information and surface normals

    std::string frame_; ///< point cloud frame
    ros::Subscriber cloud_sub_; ///< ROS subscriber for point cloud messages

    ros::Publisher grasps_pub_; ///< ROS publisher for grasp list messages
    ros::Publisher grasps_rviz_pub_; ///< ROS publisher for grasps in rviz (visualization)
    ros::Publisher obj_rects_pub_;

    bool use_rviz_; ///< if rviz is used for visualization instead of PCL

    std::shared_ptr<YoloDetector> yoloDetector;
    gpd::GraspDetectorPointNet* grasp_detector_; ///< used to run the GPD algorithm
    GraspPlotter* rviz_plotter_; ///< used to plot detected grasps in rviz
};


#endif /* GRASP_DETECTION_YOLO_ */
