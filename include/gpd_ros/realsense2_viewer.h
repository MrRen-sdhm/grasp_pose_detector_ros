#include <utility>

//
// Created by sdhm on 7/18/19.
// 读取sensor_msgs/Image以及sensor_msgs/CameraInfo, 通过深度图和相机参数创建点云, 显示和保存和彩色图像
// 获得的点云为有组织的！
//

#pragma once

#include <string>
#include <utility>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

class RealsenseReceiver
{
public:
    cv::Mat color, depth;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

private:
    std::mutex lock;

    const std::string topicColor, topicDepth;
    const bool useExact, useCompressed;

    bool updateImage, updateCloud;
    bool running;
    const size_t queueSize;

    cv::Mat cameraMatrixColor, cameraMatrixDepth;
    cv::Mat lookupX, lookupY;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner;
    image_transport::ImageTransport it;
    image_transport::SubscriberFilter *subImageColor, *subImageDepth;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

    message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
    message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

    std::thread cloudReceiverThread;

    std::vector<int> params;

public:
    RealsenseReceiver(const ros::NodeHandle& node, std::string topicColor, std::string topicDepth, const bool useExact, const bool useCompressed)
            : topicColor(std::move(topicColor)), topicDepth(std::move(topicDepth)), useExact(useExact), useCompressed(useCompressed),
              updateImage(false), updateCloud(false), running(false), queueSize(5), nh(node), spinner(0), it(nh)
    {
        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
        cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(100);
        params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        params.push_back(1);
        params.push_back(cv::IMWRITE_PNG_STRATEGY);
        params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
        params.push_back(0);
    }

    void run()
    {
        start();
    }

private:
    void start()
    {
        running = true;

        std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
        std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

        image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
        subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
        subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
        subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
        subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

        if(useExact)
        {
            syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
            syncExact->registerCallback(boost::bind(&RealsenseReceiver::callback, this, _1, _2, _3, _4));
        }
        else
        {
            syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
            syncApproximate->registerCallback(boost::bind(&RealsenseReceiver::callback, this, _1, _2, _3, _4));
        }

        spinner.start();

        std::chrono::milliseconds duration(1);
        while(!updateImage || !updateCloud)
        {
            if(!ros::ok())
            {
                return;
            }
            std::this_thread::sleep_for(duration);
        }
        cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        cloud->height = color.rows;
        cloud->width = color.cols;
        cloud->is_dense = false;
        cloud->points.resize(cloud->height * cloud->width);
        createLookup(this->color.cols, this->color.rows);

        cloudReceiverThread = std::thread(&RealsenseReceiver::cloudReceiver, this); // 获取和生成点云
    }

    void stop()
    {
        spinner.stop();

        if(useExact)
        {
            delete syncExact;
        }
        else
        {
            delete syncApproximate;
        }

        delete subImageColor;
        delete subImageDepth;
        delete subCameraInfoColor;
        delete subCameraInfoDepth;

        running = false;
    }

    void callback(const sensor_msgs::Image::ConstPtr& imageColor, const sensor_msgs::Image::ConstPtr& imageDepth,
                  const sensor_msgs::CameraInfo::ConstPtr& cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr& cameraInfoDepth)
    {
        cv::Mat color, depth;

        readCameraInfo(cameraInfoColor, cameraMatrixColor);
        readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
        readRgbImage(imageColor, color);
        readDepthImage(imageDepth, depth);

        lock.lock();
        this->color = color;
        this->depth = depth;
        updateImage = true;
        updateCloud = true;
        lock.unlock();

//        ROS_INFO("Received new image and depth.");
    }

    void cloudReceiver()
    {
        cv::Mat color, depth;

        lock.lock();
        color = this->color;
        depth = this->depth;
        updateCloud = false;
        lock.unlock();

        createCloud(depth, color, cloud);

        ros::Rate rate(15);
        for(; running && ros::ok();)
        {
            if(updateCloud)
            {
                lock.lock();
                color = this->color;
                depth = this->depth;
                updateCloud = false;
                lock.unlock();

                createCloud(depth, color, cloud);
            }

//            ros::spinOnce();
            rate.sleep();
        }

        stop();
    }

    void readRgbImage(const sensor_msgs::Image::ConstPtr& msgImage, cv::Mat &image) const
    {
        cv_bridge::CvImageConstPtr pCvImage;
        pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
        cv::cvtColor(pCvImage->image, image, cv::COLOR_BGR2RGB);
    }

    void readDepthImage(const sensor_msgs::Image::ConstPtr& msgImage, cv::Mat &image) const
    {
        cv_bridge::CvImageConstPtr pCvImage;
        pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
        pCvImage->image.copyTo(image);
    }

    void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& cameraInfo, cv::Mat &cameraMatrix) const
    {
        double *itC = cameraMatrix.ptr<double>(0, 0);
        for(size_t i = 0; i < 9; ++i, ++itC)
        {
            *itC = cameraInfo->K[i];
        }
    }

    void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const {
        const float badPoint = std::numeric_limits<float>::quiet_NaN();

#pragma omp parallel for
        for (int r = 0; r < depth.rows; ++r) {
            pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
            const uint16_t *itD = depth.ptr<uint16_t>(r);
            const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
            const float y = lookupY.at<float>(0, r);
            const float *itX = lookupX.ptr<float>();

            for (size_t c = 0; c < (size_t) depth.cols; ++c, ++itP, ++itD, ++itC, ++itX) {
                register const float depthValue = *itD / 1000.0f;
                // Check for invalid measurements
                if (*itD == 0) {
                    // not valid
                    itP->x = itP->y = itP->z = badPoint;
                    itP->rgba = 0;
                    continue;
                }
                itP->z = depthValue;
                itP->x = *itX * depthValue;
                itP->y = y * depthValue;
                itP->b = itC->val[0];
                itP->g = itC->val[1];
                itP->r = itC->val[2];
                itP->a = 255;
            }
        }
    }

    void createLookup(size_t width, size_t height)
    {
        const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
        const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
        const float cx = cameraMatrixColor.at<double>(0, 2);
        const float cy = cameraMatrixColor.at<double>(1, 2);
        float *it;

        lookupY = cv::Mat(1, height, CV_32F);
        it = lookupY.ptr<float>();
        for(size_t r = 0; r < height; ++r, ++it)
        {
            *it = (r - cy) * fy;
        }

        lookupX = cv::Mat(1, width, CV_32F);
        it = lookupX.ptr<float>();
        for(size_t c = 0; c < width; ++c, ++it)
        {
            *it = (c - cx) * fx;
        }
    }
};

#define R2_DEFAULT_NS            "camera"

#define R2_TOPIC_IMAGE_RAW       "/image_raw"
#define R2_TOPIC_IMAGE_COLOR     "/color"
#define R2_TOPIC_IMAGE_DEPTH     "/depth"
#define R2_TOPIC_ALIGNED_DEPTH   "/aligned_depth_to_color"

#define R2_TOPIC_COMPRESSED      "/compressed"
#define R2_TOPIC_INFO            "/camera_info"

