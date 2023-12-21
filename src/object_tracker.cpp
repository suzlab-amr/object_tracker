// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#include <object_tracker/object_tracker.hpp>
#include <object_tracker/state_tracker.hpp>
#include <hungarian_algorithm/Hungarian.h>
#include <kalman_filter/kalman_filter.hpp>

#include <string>
#include <cmath>
#include <vector>
#include <utility>
#include <unordered_set>
#include <limits>
#include <cstdint>
#include <json/json.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <object_msgs/ObjectArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace object_tracker
{

ObjectTracker::ObjectTracker() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
    // Get parameters.
    std::string objects_in_topic, objects_out_topic;
    double prediction_position_variance, prediction_velocity_variance;
    double observation_position_variance, observation_velocity_variance;
    double initial_position_variance, initial_velocity_variance;
    pnh_.param<std::string>("objects_in", objects_in_topic, "/objects_in");
    pnh_.param<std::string>("objects_out", objects_out_topic, "/objects_out");
    pnh_.param<std::string>("tracking_frame", tracking_frame_, "map");
    pnh_.param<float>("double_cost_distance", kd_, 0.5);
    pnh_.param<float>("max_track_speed", max_track_speed_, 3.0);
    pnh_.param<double>("prediction_position_variance", prediction_position_variance, 0.25);
    pnh_.param<double>("prediction_velocity_variance", prediction_velocity_variance, 0.25);
    pnh_.param<double>("observation_position_variance", observation_position_variance, 1e-4);
    pnh_.param<double>("observation_velocity_variance", observation_velocity_variance, 1e-4);
    pnh_.param<double>("initial_position_variance", initial_position_variance, 1e-4);
    pnh_.param<double>("initial_velocity_variance", initial_velocity_variance, 1.0);

    // Initialize variables.
    log2_3_ = std::log2(3);
    tracking_id_ = 0;
    init_kalman_filter_matrices(
        prediction_position_variance,
        prediction_velocity_variance,
        observation_position_variance,
        observation_velocity_variance,
        initial_position_variance,
        initial_velocity_variance
    );

    // Set up subscribers and publishers.
    objects_sub_ = nh_.subscribe(objects_in_topic, 1, &ObjectTracker::objects_callback, this);
    objects_pub_ = nh_.advertise<object_msgs::ObjectArray>(objects_out_topic, 1);
}

void ObjectTracker::init_kalman_filter_matrices(const double wp, const double wv, const double vp, const double vv, const double ip, const double iv)
{
    // State transition matrix with constant values.
    A_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

    // State transition matrix multiplied by time step.
    A_dt_ << 0, 0, 1, 0,
             0, 0, 0, 1,
             0, 0, 0, 0,
             0, 0, 0, 0;

    // Observation matrix with constant values.
    C_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

    // Process noise covariance matrix.
    Q_ << wp,  0,  0,  0,
           0, wp,  0,  0,
           0,  0, wv,  0,
           0,  0,  0, wv;

    // Observation noise covariance matrix.
    R_ << vp,  0,  0,  0,
           0, vp,  0,  0,
           0,  0, vv,  0,
           0,  0,  0, vv;

    // Initial state covariance matrix.
    P0_ << ip,  0,  0,  0,
            0, ip,  0,  0,
            0,  0, iv,  0,
            0,  0,  0, iv;

    // Initial state vector.
    x0_ << 0, 0, 0, 0;
}

uint8_t ObjectTracker::get_new_id()
{
    while (ros::ok() && used_ids_.find(tracking_id_) != used_ids_.end())
    {
        tracking_id_++;
        if (tracking_id_ == std::numeric_limits<uint8_t>::max())
            tracking_id_ = 0;
    }
    used_ids_.insert(tracking_id_);
    return tracking_id_;
}

void ObjectTracker::objects_callback(const object_msgs::ObjectArray::ConstPtr& msg)
{
    // Get time step.
    double dt = (msg->header.stamp - last_time_).toSec();

    // Copy object array.
    auto object_array = *msg;

    // Limit number of objects.
    while (object_array.objects.size() > (size_t)std::numeric_limits<uint8_t>::max())
        object_array.objects.pop_back();

    // Transform object positions to tracking frame.
    if (tracking_frame_ != object_array.header.frame_id)
    {
        // Get transform from lidar to map.
        geometry_msgs::TransformStamped lidar2map;
        try
        {
            lidar2map = tf_buffer_.lookupTransform(
                tracking_frame_,
                msg->header.frame_id,
                ros::Time(0)
            );
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN_DELAYED_THROTTLE(5, "%s", ex.what());
            return;
        }

        // Transform object positions.
        for (size_t i = 0; i < object_array.objects.size(); i++)
        {
            geometry_msgs::Point lidar_point, map_point;
            lidar_point.x = object_array.objects[i].pose.position.x;
            lidar_point.y = object_array.objects[i].pose.position.y;
            lidar_point.z = object_array.objects[i].pose.position.z;
            tf2::doTransform(lidar_point, map_point, lidar2map);
            object_array.objects[i].pose.position.x = map_point.x;
            object_array.objects[i].pose.position.y = map_point.y;
            object_array.objects[i].pose.position.z = map_point.z;
        }
        object_array.header.frame_id = tracking_frame_;
    }

    // Get cost matrix for Hungarian algorithm.
    std::vector<std::vector<double>> cost_matrix;
    for (auto current_state : object_array.objects)
    {
        std::vector<double> cost_row;
        for (size_t i = 0; i < last_trackers_.size(); i++)
        {
            // Predict state of tracked object.
            Eigen::Vector4d x_pred = last_trackers_[i].second.predict(dt);

            double distance_pow =
                (current_state.pose.position.x - x_pred[0]) * (current_state.pose.position.x - x_pred[0])
                + (current_state.pose.position.y - x_pred[1]) * (current_state.pose.position.y - x_pred[1]);

            cost_row.push_back(distance_pow / (kd_ * (1 + pow(sqrt(distance_pow) / kd_, log2_3_))));
        }
        cost_matrix.push_back(cost_row);
    }

    // Solve assignment problem.
    std::vector<int> best_combinations;
    if (!last_trackers_.empty() && !msg->objects.empty())
    {
        HungarianAlgorithm hungarian_algorithm;
        hungarian_algorithm.Solve(cost_matrix, best_combinations);
    }
    else if (last_trackers_.empty() && !msg->objects.empty())
        best_combinations = std::vector<int>(msg->objects.size(), -1);

    // Update state trackers.
    std::vector<std::pair<StateTracker, KalmanFilter>> trackers;
    std::unordered_set<uint8_t> updated_trackers;
    for (size_t i = 0; i < best_combinations.size(); i++)
    {
        // Create position stamped.
        StateTracker::Vector3Stamped position_stamped;
        position_stamped.time_stamp = msg->header.stamp.toSec();
        position_stamped.position = {
            object_array.objects[i].pose.position.x,
            object_array.objects[i].pose.position.y,
            0
        };

        // Try adding state to existing state tracker.
        if (best_combinations[i] != -1
            && last_trackers_[best_combinations[i]].first.add_state(position_stamped))
        {
            // Update kalman filter.
            Eigen::Vector4d y = {
                position_stamped.position.x,
                position_stamped.position.y,
                last_trackers_[best_combinations[i]].first.get_velocity().x,
                last_trackers_[best_combinations[i]].first.get_velocity().y
            };
            last_trackers_[best_combinations[i]].second.update(y);

            trackers.push_back(last_trackers_[best_combinations[i]]);
            updated_trackers.insert((uint8_t)last_trackers_[best_combinations[i]].first.get_id());
            continue;
        }

        // Add new tracker.
        trackers.push_back(
            std::pair<StateTracker, KalmanFilter>(
                StateTracker(-1, max_track_speed_, 0),
                KalmanFilter(A_, A_dt_, C_, Q_, R_)
            )
        );
        trackers.back().first.add_state(position_stamped);
        x0_ << position_stamped.position.x, position_stamped.position.y, 0, 0;
        trackers.back().second.set_states(x0_, P0_);
    }

    // Remove unused ids.
    for (auto last_tracker : last_trackers_)
        if (updated_trackers.find(last_tracker.first.get_id()) == updated_trackers.end())
            used_ids_.erase(last_tracker.first.get_id());

    // Set new id for new state trackers.
    for (size_t i = 0; i < trackers.size(); i++)
        if (trackers[i].first.get_id() == -1)
            trackers[i].first.set_id(get_new_id());

    // Update object array.
    for (size_t i = 0; i < trackers.size(); i++)
    {
        Json::Value json;
        json["tracking_id"] = trackers[i].first.get_id();
        object_array.objects[i].json_string = json.toStyledString();

        Eigen::Vector4d x = trackers[i].second.get_updated_state();
        object_array.objects[i].pose.position.x = x[0];
        object_array.objects[i].pose.position.y = x[1];

        geometry_msgs::Vector3 velocity;
        velocity.x = x[2];
        velocity.y = x[3];

        object_array.objects[i].twist.linear.x = sqrt(velocity.x * velocity.x + velocity.y * velocity.y);

        double yaw = std::atan2(velocity.y, velocity.x);
        object_array.objects[i].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));
    }

    // Publish object array.
    objects_pub_.publish(object_array);

    // Preserve info for next callback.
    last_time_ = msg->header.stamp;
    last_trackers_ = trackers;
}

} // namespace object_tracker
