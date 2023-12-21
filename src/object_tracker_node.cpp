// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#include <object_tracker/object_tracker.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_tracker");
    object_tracker::ObjectTracker object_tracker;
    ros::spin();

    return 0;
}
