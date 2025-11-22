#ifndef GROUP14_UTILS_HPP
#define GROUP14_UTILS_HPP

#include "geometry_msgs/msg/point_stamped.hpp"

namespace group14
{
    // Point in 2D space (already transformed?)
    struct Point
    {
        double x;
        double y;
    };

    struct RangePoint
    {
        float range;
        float angle;
        int index;
        geometry_msgs::msg::PointStamped point;

        RangePoint(float range_, float angle_, int index_, geometry_msgs::msg::PointStamped point_)
            : range(range_), angle(angle_), index(index_), point(point_) {}
    };

    constexpr double PI = 3.14159265358979323846;
}

#endif
