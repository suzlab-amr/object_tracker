// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#ifndef STATE_TRACKER_HPP_
#define STATE_TRACKER_HPP_

#include <vector>
#include <cstdint>


class StateTracker
{
public:
    /**
     * \param id ID of the object.
     * \param max_speed Maximum speed [m/s] considered when judging whether the object is the same or not.
     * \param n_smoothing Number of logs skipped when calculating velocity.
    */
    template <class T, class U, class V>
    StateTracker (const T id, const U max_speed, const V n_smoothing=0)
    {
        id_ = (int)id;
        max_speed_ = (float)max_speed;
        n_smoothing_ = (uint32_t)std::max((int)n_smoothing, 0);
    }

    /**
     * \brief Vector3.
    */
    struct Vector3
    {
        double x;
        double y;
        double z;

        Vector3 operator+(const Vector3 &rhs) const
        {
            return {x + rhs.x, y + rhs.y, z + rhs.z};
        }

        Vector3 operator-(const Vector3 &rhs) const
        {
            return {x - rhs.x, y - rhs.y, z - rhs.z};
        }

        Vector3 operator*(const double &rhs) const
        {
            return {x * rhs, y * rhs, z * rhs};
        }

        friend Vector3 operator*(const double &lhs, const Vector3 &rhs)
        {
            return {lhs * rhs.x, lhs * rhs.y, lhs * rhs.z};
        }

        Vector3 operator/(const double &rhs) const
        {
            return {x / rhs, y / rhs, z / rhs};
        }
    };

    /**
     * \brief Vector3 with time stamp.
    */
    struct Vector3Stamped
    {
        double time_stamp;
        Vector3 position;
    };

    /**
     * \brief Get ID.
     * \return ID.
    */
    int get_id();

    /**
     * \brief Set ID.
     * \param id ID.
    */
    template <class T>
    void set_id(const T id)
    {
        id_ = (int)id;
    }

    /**
     * \brief Get latest position.
     * \return Latest position.
    */
    Vector3 get_position();

    /**
     * \brief Get latest velocity.
     * \return Latest velocity.
    */
    Vector3 get_velocity();

    /**
     * \brief Add state.
     * \param position_stamped Position with time stamp.
     * \return True if success.
    */
    bool add_state(const Vector3Stamped &position_stamped);

private:
    /**
     * \brief ID of the object.
    */
    int id_;

    /**
     * \brief Maximum speed [m/s] considered when judging whether the object is the same or not.
    */
    float max_speed_;

    /**
     * \brief Number of logs skipped when calculating velocity.
    */
    uint32_t n_smoothing_;

    /**
     * \brief Velocity of tracked object.
    */
    Vector3 velocity_;

    /**
     * \brief Log of states with time stamps.
    */
    std::vector<Vector3Stamped> log_;
};

#endif // STATE_TRACKER_HPP_
