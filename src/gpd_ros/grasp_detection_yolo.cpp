#include <gpd_ros/grasp_detection_yolo.h>

GraspDetectionYolo::GraspDetectionYolo(ros::NodeHandle& node)
{
    printf("Init ....\n");
    cloud_camera_ = NULL;

//    std::string cfg_file;
//    node.param("config_file", cfg_file, std::string("/home/sdhm/catkin_ws/src/gpd_ros/cfg/ros_gpd_pointnet_params.cfg"));
//    grasp_detector_ = new gpd::GraspDetectorPointNet(cfg_file);
//    printf("Created GPD ....\n");
//
//    // Read input cloud and ROS topics parameters.
//    std::string cloud_topic;
//    node.param("cloud_topic", cloud_topic, std::string("/camera/depth/color/points"));
//    std::string rviz_topic;
//    node.param("rviz_topic", rviz_topic, std::string("grasps_rviz"));
//
//    if (!rviz_topic.empty()) {
//        grasps_rviz_pub_ = node.advertise<visualization_msgs::MarkerArray>(rviz_topic, 1);
//        use_rviz_ = true;
//    } else {
//        use_rviz_ = false;
//    }
//
//    // uses ROS topics to publish grasp candidates, antipodal grasps, and grasps after clustering
//    grasps_pub_ = node.advertise<gpd_ros::GraspConfigList>("clustered_grasps", 10);
//
//    cloud_sub_ = node.subscribe(cloud_topic, 1, &GraspDetectionYolo::cloud_callback, this);
//
//    rviz_plotter_ = new GraspPlotter(node, grasp_detector_->getHandSearchParameters().hand_geometry_);

    /// YoloDetector
    std::string yolo_config_filename = "/home/sdhm/yolov3-voc.cfg";
    std::string yolo_weights_filename ="/home/sdhm/yolov3-voc_23000.weights";
    yoloDetector = std::make_shared<YoloDetector>(yolo_config_filename, yolo_weights_filename);

    obj_rects_pub_ = node.advertise<std_msgs::Float64MultiArray>("juggle_rects", 10);

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

void GraspDetectionYolo::run(int loop_rate)
{
    ros::Rate rate(loop_rate); // 与采集频率接近即可
    while (ros::ok())
    {
        collect_image_cloud(); // 获取图像和点云

        std::vector<cv::RotatedRect> RotatedRects;

        RotatedRects = yoloDetector->getObjRect(image_);

        std_msgs::Float64MultiArray juggle_rects_arr;
        for (size_t i = 0; i < RotatedRects.size(); i++) {
            // 获取外接矩形实际位置
            int row = (int)RotatedRects[i].center.y;
            int col = (int)RotatedRects[i].center.x;
            float center_x = cloud_->points[row * cloud_->width + col].x;
            float center_y = cloud_->points[row * cloud_->width + col].y;
            float center_z = cloud_->points[row * cloud_->width + col].z;
            float center_angle = RotatedRects[i].angle;

            // 中心点附近寻找可用深度
            for (int i = -5; i <= 5; i++) {
                if (center_z > 0.1 && center_z < 2.0) break;
                for (int j = -5; j <= 5; j++) {
                    center_z = cloud_->points[(row+i) * cloud_->width + (col+j)].z;
                    printf("Find center z: %f\n", center_z);
                    if (center_z > 0.1 && center_z < 2.0) break;
                }
            }

            if (center_z < 0.1 || center_z > 2.0) {
                printf("row: %d col: %d center_z: %f\n", row, col, center_z);
                throw std::runtime_error("\033[0;31mCenter point's depth is not valid!\033[0m\n");
            }

            // angle为与width边的夹角, width > height 时angle为与长边夹角
            if (RotatedRects[i].size.width < RotatedRects[i].size.height) {
                center_angle += 90; // 转换为长边的角度
            }

            juggle_rects_arr.data.push_back(center_x);
            juggle_rects_arr.data.push_back(center_y);
            juggle_rects_arr.data.push_back(center_z);
            juggle_rects_arr.data.push_back(center_angle);

            printf("center x:%f y:%f z:%f angle:%f\n", center_x, center_y, center_z, center_angle);
        }

        obj_rects_pub_.publish(juggle_rects_arr); // 发布积木外接矩形信息

//        cout << "size:" << cloud_->size() << endl;
//        cout << "height:" << cloud_->height << endl;
//        cout << "width:" << cloud_->width << endl;
//
//        int row = 214, col = 317;
//        cout << cloud_->points[row * cloud_->width + col].x << "\n"
//                << cloud_->points[row * cloud_->width + col].y << endl ;

//        for (int row = 0; row < cloud_->height; ++row) { // 540
//            for (int col = 0; col < cloud_->width; ++col) { // 960
//                int indices = row * cloud_->width + col; // 二维索引
//                float depth = cloud_->points[indices].z;
//                float x = cloud_->points[indices].x;
//                float y = cloud_->points[indices].y;
//                if (depth > 0.1 && depth < 1.2){ // 确保有深度信息
//                    printf("depth[%d %d] [x:%f y:%f] (indice: %d): %f\n", row, col, x, y, indices, depth);
//                }
//            }
//        }

//        // Detect grasps in point cloud.
//        std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = detectGraspPoses();
//
//        // Visualize the detected grasps in rviz.
//        if (use_rviz_)
//        {
//            printf("Visualize the detected grasps in rviz.\n");
//            printf("frame:%s\n", frame_.c_str());
//            rviz_plotter_->drawGrasps(grasps, frame_);
//        }

        ros::spinOnce();
        rate.sleep();
    }
}


std::vector<std::unique_ptr<gpd::candidate::Hand>> GraspDetectionYolo::detectGraspPoses()
{
    // detect grasp poses
    std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps;

    // preprocess the point cloud
    grasp_detector_->preprocessPointCloud(*cloud_camera_);

    // detect grasps in the point cloud
    grasps = grasp_detector_->detectGrasps(*cloud_camera_);

    // Publish the selected grasps.
    gpd_ros::GraspConfigList selected_grasps_msg = GraspMessages::createGraspListMsg(grasps);
    grasps_pub_.publish(selected_grasps_msg);
    ROS_INFO_STREAM("Published " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.");

    return grasps;
}

void GraspDetectionYolo::collect_image_cloud() {
    ROS_INFO("Get new image and cloud...");
    image_ = receiver->color.clone();
    cloud_ = receiver->cloud;

    Eigen::Matrix3Xd view_points(3, 1);
    view_points.col(0) = view_point_;

    cloud_camera_ = new gpd::util::Cloud(receiver->cloud, 0, view_points);
    ROS_INFO_STREAM("Process cloud with " << cloud_camera_->getCloudProcessed()->size() << " points.");
}

void GraspDetectionYolo::cloud_callback(const sensor_msgs::PointCloud2& msg)
{
    frame_ = msg.header.frame_id;
}

YoloDetector::YoloDetector(std::string config_file, std::string weights_file) {
    net_ = cv::dnn::readNetFromDarknet(config_file, weights_file);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

std::vector<cv::RotatedRect> YoloDetector::getObjRect(cv::Mat image) {
    cv::Mat blob;
    std::vector<cv::Rect> obj_boxes;
    std::vector<cv::RotatedRect> RotatedRects;
    cv::dnn::blobFromImage(image, blob, 1/255.0, cvSize(inpWidth_, inpHeight_), cv::Scalar(0,0,0), true, false); // Create a 4D blob from a frame.
    net_.setInput(blob); // Sets the input to the network
    std::vector<cv::Mat> outs;
    net_.forward(outs, getOutputsNames(net_)); // Runs the forward pass to get output of the output layers
//    obj_boxes = postprocess(image, outs, confThreshold_, nmsThreshold_, get_classes_vec()); // Remove the bounding boxes with low confidence
//    return obj_boxes;
    RotatedRects = postprocess(image, outs, confThreshold_, nmsThreshold_, get_classes_vec()); // Remove the bounding boxes with low confidence
    return RotatedRects;
}

// Remove the bounding boxes with low confidence using non-maxima suppression
std::vector<cv::RotatedRect> YoloDetector::postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs, float confThreshold,
                                                float nmsThreshold, std::vector<std::string> classes)
{
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols * 1.5);
                int height = (int)(data[3] * frame.rows * 1.5);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.emplace_back(left, top, width, height);
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

    int bottleNum = 0; // 瓶子个数
    std::vector<cv::Rect> obj_boxes;
    std::vector<cv::RotatedRect> RotatedRects;
    cv::Mat frame_copy(frame);
    for (size_t i = 0; i < indices.size(); ++i) // 处理各检测到的物体
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];

        if (box.height * box.width < 90*90) continue; // FIXME: 滤除过小的物体

        cv::Mat img_roi = frame.clone()(box);
        if (DEBUG) cv::imshow("roi", img_roi);

        cv::Mat img_hsv;
        cv::cvtColor(img_roi, img_hsv, CV_BGR2HSV);
        if (DEBUG) cv::imshow("hsv", img_hsv);

        cv::Mat mask = cv::Mat::zeros(img_hsv.rows, img_hsv.cols, CV_8U); // 掩码

        for(int r = 0; r < img_hsv.rows; ++r)
        {
            auto *itM = mask.ptr<uint8_t>(r);
            const cv::Vec3b *itH = img_hsv.ptr<cv::Vec3b>(r);

            for(int c = 0; c < img_hsv.cols; ++c, ++itM, ++itH)
            {
                if (itH->val[0] < 35 && itH->val[2] > 100) {
                    *itM = 255;
                }
            }
        }

        if (DEBUG) cv::imshow("mask", mask);

        /// 轮廓查找
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        findContours(mask, contours, hierarchy, cv::RETR_TREE, cv:: CHAIN_APPROX_SIMPLE);

        if (contours.empty()) continue;

        int index = 0;
        for (; index >= 0; index = hierarchy[index][0]) {
            drawContours(img_roi, contours, index, cv::Scalar(0, 255, 255), cv::FILLED, 8, hierarchy);
        }
//        cv::imshow("roi_Contours", img_roi);

        /// 查找最大轮廓
        double max_area = 0;
        int maxAreaIdx = 0;
        for (int index = contours.size() - 1; index >= 0; index--)
        {
            double tmp_area = fabs(contourArea(contours[index]));
            if (tmp_area > max_area)
            {
                max_area = tmp_area;
                maxAreaIdx = index; //记录最大轮廓的索引号
            }
        }

        std::vector<cv::Point> contourlist; // 轮廓列表
        contourlist = contours[maxAreaIdx]; // 最大轮廓

        /// 最大轮廓的最小外接矩形
        cv::RotatedRect rect = minAreaRect(contourlist);
        if (DEBUG) std::cout << "minAreaRect: center:" << rect.center << " angle: " << rect.angle << " size: " << rect.size << std::endl;

        cv::Point2f P[4];
        rect.points(P);
        for(int j=0; j <= 3; j++) {
            line(img_roi, P[j], P[(j + 1) % 4], cv::Scalar(0, 255 ,0), 2);
        }
        cv::circle(img_roi, rect.center, 1, cv::Scalar(0, 0, 255), 2);

        if (DEBUG) cv::imshow("roi_minAreaRect", img_roi);

        cv::RotatedRect rect_out(rect); // 获取整张图片下的中心位置及角度
        rect_out.center.x += box.x;
        rect_out.center.y += box.y;
        RotatedRects.push_back(rect_out);

        if (DEBUG) std::cout << "minAreaRectOut: center:" << rect_out.center << " angle: " << rect_out.angle << " size: " << rect_out.size << std::endl;

        // 获取各目标位置
        if (classes[classIds[idx]] == "bottle" && confidences[idx] > 0.4) {
            std::cout << box << std::endl;
            obj_boxes.push_back(box);
            bottleNum ++;
        }

        if (DEBUG) drawPred(classIds[idx], confidences[idx], box.x, box.y, box.x + box.width, box.y + box.height, frame_copy, classes); // 画边框
    }

    if (DEBUG) cv::imshow("image", frame_copy);
    if (DEBUG) cv::waitKey(0);

//    return obj_boxes;
    return RotatedRects;
}

// Draw the predicted bounding box
void YoloDetector::drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame, std::vector<std::string> classes)
{
    //Draw a rectangle displaying the bounding box
    rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 2); // 边框

    //Get the label for the class name and its confidence
    std::string label = cv::format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)(classes.size()));
        label = classes[classId] + ":" + label;
    }

    //Display the label at the top of the bounding box
    int baseLine;
    cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = std::max(top, labelSize.height);
    rectangle(frame, cv::Point(left-1, top - round(1.2*labelSize.height)), // 标题背景
              cv::Point(left + round(1.2*labelSize.width), top + baseLine), cv::Scalar(255, 178, 50), cv::FILLED);
    putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0),1,CV_AA); // 标题

//    rectangle(frame, cv::Point((left+right)/2-2, (top+bottom)/2-2), // 中心点
//              cv::Point((left+right)/2+2, (top+bottom)/2+2), cv::Scalar(0, 0, 255), cv::FILLED);
//    rectangle(frame, cv::Point((left+right)/2 - (right-left)/6, (top+bottom)/2 - (bottom-top)/6),
//              cv::Point((left+right)/2 + (right-left)/6, (top+bottom)/2 + (bottom-top)/6), cv::Scalar(0, 255, 0));
}

// Get the names of the output layers
std::vector<cv::String> YoloDetector::getOutputsNames(const cv::dnn::Net& net)
{
    static std::vector<cv::String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        std::vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        std::vector<cv::String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detect_grasps_yolo");
    ros::NodeHandle node("~");

    if(!ros::ok())
    {
        return 1;
    }

    int loop_rate = 15; // 与采集频率接近即可

    GraspDetectionYolo grasp_detection(node);
    grasp_detection.run(loop_rate);

    ros::shutdown();
    return 0;
}
