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

    struct Circle
    {
        float x_center;
        float y_center;
        float r;

        Circle(float x_center_, float y_center_, float r_)
            : x_center(x_center_), y_center(y_center_), r(r_) {}
    };

    constexpr double PI = 3.14159265358979323846;
}

#endif
