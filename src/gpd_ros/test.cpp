#include <gpd_ros/test.h>

GraspDetectionPointnet::GraspDetectionPointnet(ros::NodeHandle& node)
{
    ROS_INFO("Starting grasp detection server...");
    cloud_camera_ = NULL;

    std::string cfg_file;
    node.param("config_file", cfg_file, std::string("/home/sdhm/catkin_ws/src/gpd_ros/cfg/python_classifier_params.cfg"));
    printf("[INFO] Config file: %s\n", cfg_file.c_str());

    grasp_detector_ = new gpd::GraspDetectorPointNet(cfg_file);

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

    // for marker plot in rviz
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

    receiver = std::make_shared<RealsenseReceiver>(node, topicColor, topicDepth, 10, useExact, useCompressed);

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

/// Python函数无法在单独的线程中运行，会出现程序锁死的情况，Service的实现应该是另起线程了，导致C++调用Python卡死，
/// 所以这里在主函数中使用while语句检测抓取，通过改变标志位获取数据
bool GraspDetectionPointnet::detectGrasps(gpd_ros::detect_grasps::Request& req, gpd_ros::detect_grasps::Response& res)
{
    run_flag_ = true; // start detect grasps
//    printf("[DEBUG] start detect\n");
//
//    printf("[DEBUG] run_done: %d\n", run_done_);

    while(true) {  // wait for detecting grasps done
        usleep(5000); // sleep for run_done flag to update

        if(run_done_) {
            run_done_ = false; // reset done flag

            if (!grasps.empty()) {
                // Response the detected grasps.
                gpd_ros::GraspConfigList selected_grasps_msg = GraspMessages::createGraspListMsg(grasps);
                res.grasp_configs = selected_grasps_msg;
                ROS_INFO_STREAM("Detected " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.");

                return true;
            } else return false;
        }
    }
}

bool GraspDetectionPointnet::detectGraspsFunc()
{
    if (run_flag_) {
        run_flag_ = false;

        // get cloud from realsense receiver
        collect_cloud();

        // preprocess the point cloud
        grasp_detector_->preprocessPointCloud(*cloud_camera_);

        // detect grasps in the point cloud
        grasps = grasp_detector_->detectGrasps(*cloud_camera_);

        run_done_ = true;

        // Visualize the detected grasps in rviz.
        if (use_rviz_ && !grasps.empty()) {
            ROS_INFO_STREAM("Visualize the detected grasps in rviz.");
            rviz_plotter_->drawGrasps(grasps, frame_);

//            cout << "[INFO] Grasps[0] position:" << endl << grasps[0]->getPosition() << endl;

            return true;
        }
    }

    return false;
}

bool GraspDetectionPointnet::detectGraspsTest()
{
    std::string pcd_filename = "/home/sdhm/200109_00_cloud.pcd";

    Eigen::Matrix3Xd view_points(3, 1);
    view_points.col(0) = view_point_;

    cloud_camera_ = new gpd::util::Cloud(pcd_filename, view_points);
    ROS_INFO_STREAM("Process cloud with " << cloud_camera_->getCloudProcessed()->size() << " points.");

    // preprocess the point cloud
    grasp_detector_->preprocessPointCloud(*cloud_camera_);

    // detect grasps in the point cloud
    grasp_detector_->detectGrasps(*cloud_camera_);

    return false;
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
    ros::init(argc, argv, "detect_grasps_server_pointnet_realsense");

    ros::NodeHandle node("~");

    int loop_rate = 15; // 与采集频率接近即可

    GraspDetectionPointnet grasp_detection(node);

    grasp_detection.detectGraspsTest();
    grasp_detection.detectGraspsTest();

//    grasp_detection.run(loop_rate);

    ros::shutdown();
    return 0;
}