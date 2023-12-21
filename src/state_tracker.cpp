// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#include <object_tracker/state_tracker.hpp>

#include <cmath>
#include <limits>
#include <cstdint>


int StateTracker::get_id()
{
    return id_;
}

StateTracker::Vector3 StateTracker::get_position()
{
    if (log_.empty())
        return {0, 0, 0};
    return log_.back().position;
}

StateTracker::Vector3 StateTracker::get_velocity()
{
    if (log_.empty())
        return {0, 0, 0};
    return velocity_;
}

bool StateTracker::add_state(const Vector3Stamped &position_stamped)
{
    // Check if the objects speed is within the maximum speed.
    if (!log_.empty())
    {
        const Vector3 dp = position_stamped.position - log_.back().position;
        const double dt = position_stamped.time_stamp - log_.back().time_stamp;

        if (dt <= 0)
            return false;

        const Vector3 velocity = dp / dt;
        const double speed = sqrt(
            velocity.x*velocity.x
            + velocity.y*velocity.y
            + velocity.z*velocity.z
        );

        if (speed > max_speed_)
            return false;
    }

    log_.push_back(position_stamped);

    if ((uint32_t)log_.size() < 2+n_smoothing_)
    {
        velocity_ = {0, 0, 0};
        return true;
    }

    // Calculate velocity.
    const Vector3 dp = log_.back().position - log_.front().position;
    const double dt = log_.back().time_stamp - log_.front().time_stamp;
    velocity_ = dp / dt;

    log_.erase(log_.begin());

    return true;
}
