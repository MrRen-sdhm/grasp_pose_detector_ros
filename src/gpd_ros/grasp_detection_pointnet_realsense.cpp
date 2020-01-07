#include <gpd_ros/grasp_detection_pointnet_realsense.h>

GraspDetectionPointnet::GraspDetectionPointnet(ros::NodeHandle& node)
{
    printf("Init ....\n");
    cloud_camera_ = NULL;

    std::string cfg_file;
    node.param("config_file", cfg_file, std::string("/home/sdhm/catkin_ws/src/gpd_ros/cfg/python_classifier_params.cfg"));
    grasp_detector_ = new gpd::GraspDetectorPointNet(cfg_file);
    printf("Created GPD ....\n");

    // Read input cloud and ROS topics parameters.
    std::string cloud_topic;
    node.param("cloud_topic", cloud_topic, std::string("/camera/depth/color/points"));
    std::string rviz_topic;
    node.param("rviz_topic", rviz_topic, std::string("grasps_rviz"));

    if (!rviz_topic.empty()) {
        grasps_rviz_pub_ = node.advertise<visualization_msgs::MarkerArray>(rviz_topic, 1);
        use_rviz_ = true;
    } else {
        use_rviz_ = false;
    }

    // uses ROS topics to publish grasp candidates, antipodal grasps, and grasps after clustering
    grasps_pub_ = node.advertise<gpd_ros::GraspConfigList>("clustered_grasps", 10);

    cloud_sub_ = node.subscribe(cloud_topic, 1, &GraspDetectionPointnet::cloud_callback, this);

    rviz_plotter_ = new GraspPlotter(node, grasp_detector_->getHandSearchParameters().hand_geometry_);

    /// Realsense cloud and image receiver
    std::string ns = R2_DEFAULT_NS;
    std::string topicColor = R2_TOPIC_IMAGE_COLOR R2_TOPIC_IMAGE_RAW;
    std::string topicDepth = R2_TOPIC_ALIGNED_DEPTH R2_TOPIC_IMAGE_RAW;
    bool useExact = true;
    bool useCompressed = false;
    topicColor = "/" + ns + topicColor;
    topicDepth = "/" + ns + topicDepth;

    receiver = std::make_shared<RealsenseReceiver>(node, topicColor, topicDepth, useExact, useCompressed);

    ROS_INFO("Starting realsense receiver...");
    receiver->run();
}

void GraspDetectionPointnet::run(int loop_rate)
{
    ros::Rate rate(loop_rate); // 与采集频率接近即可
    while (ros::ok())
    {
        collect_cloud(); // 获取点云

        // Detect grasps in point cloud.
        std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = detectGraspPoses();

        // Visualize the detected grasps in rviz.
        if (use_rviz_ && !grasps.empty())
        {
            printf("Visualize the detected grasps in rviz.\n");
            printf("frame:%s\n", frame_.c_str());
            rviz_plotter_->drawGrasps(grasps, frame_);
        }

        ros::spinOnce();
        rate.sleep();
    }
}


std::vector<std::unique_ptr<gpd::candidate::Hand>> GraspDetectionPointnet::detectGraspPoses()
{
    // detect grasp poses
    std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps;

    // preprocess the point cloud
    grasp_detector_->preprocessPointCloud(*cloud_camera_);

    // detect grasps in the point cloud
    grasps = grasp_detector_->detectGrasps(*cloud_camera_);

    if (!grasps.empty()) {
        // Publish the selected grasps.
        gpd_ros::GraspConfigList selected_grasps_msg = GraspMessages::createGraspListMsg(grasps);
        grasps_pub_.publish(selected_grasps_msg);
        ROS_INFO_STREAM("Published " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.");
    }

    return grasps;
}

void GraspDetectionPointnet::collect_cloud() {
    Eigen::Matrix3Xd view_points(3, 1);
    view_points.col(0) = view_point_;

    cloud_camera_ = new gpd::util::Cloud(receiver->cloud, 0, view_points);
    ROS_INFO_STREAM("Process cloud with " << cloud_camera_->getCloudProcessed()->size() << " points.");
}

void GraspDetectionPointnet::cloud_callback(const sensor_msgs::PointCloud2& msg)
{
    frame_ = msg.header.frame_id;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detect_grasps_pointnet_realsense");

    //    ros::NodeHandle node("~");
    ros::NodeHandle node; // namespace not used

    if(!ros::ok())
    {
        return 1;
    }

    int loop_rate = 15; // 与采集频率接近即可

    GraspDetectionPointnet grasp_detection(node);
    grasp_detection.run(loop_rate);

    ros::shutdown();
    return 0;
}
