// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#ifndef OBJECT_TRACKER_HPP_
#define OBJECT_TRACKER_HPP_

#include <object_tracker/state_tracker.hpp>
#include <hungarian_algorithm/Hungarian.h>
#include <kalman_filter/kalman_filter.hpp>

#include <vector>
#include <utility>
#include <unordered_set>
#include <cstdint>

#include <eigen3/Eigen/Core>

#include <ros/ros.h>
#include <object_msgs/ObjectArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


namespace object_tracker
{

class ObjectTracker
{
public:
    ObjectTracker();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber objects_sub_;
    ros::Publisher objects_pub_;

    ros::Time last_time_;

    tf2_ros::TransformListener tf_listener_;
    tf2_ros::Buffer tf_buffer_;

    std::string tracking_frame_;

    uint8_t tracking_id_;
    std::unordered_set<uint8_t> used_ids_;

    float kd_;
    float max_track_speed_;

    double log2_3_;

    std::vector<std::pair<StateTracker, KalmanFilter>> last_trackers_;

    Eigen::Matrix4d A_, A_dt_, C_, Q_, R_;
    Eigen::Matrix4d P0_;
    Eigen::Vector4d x0_;

    HungarianAlgorithm hungarian_algorithm_;

    void init_kalman_filter_matrices(const double wp, const double wv, const double vp, const double vv, const double ip, const double iv);
    uint8_t get_new_id();
    void objects_callback(const object_msgs::ObjectArray::ConstPtr& msg);
};

} // namespace object_tracker

#endif // OBJECT_TRACKER_HPP_

