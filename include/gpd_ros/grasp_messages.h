#ifndef GRASP_MESSAGES_H_
#define GRASP_MESSAGES_H_

#include <eigen_conversions/eigen_msg.h>

#include <gpd/candidate/hand.h>

#include <gpd_ros/GraspConfig.h>
#include <gpd_ros/GraspConfigList.h>

namespace GraspMessages
{
    gpd_ros::GraspConfigList createGraspListMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std_msgs::Header& header);
    gpd_ros::GraspConfigList createGraspListMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands);

    gpd_ros::GraspConfig convertToGraspMsg(const gpd::candidate::Hand& hand);
};

#endif /* GRASP_MESSAGES_H_ */
