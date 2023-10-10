#pragma once

#include "AMPCore.h"

class Helper{
    public:
        /**
         * @brief returns true if a point is on a line segment.
         * 
         * @param start start of line segment
         * @param end end of line segment
         * @param point point to check
         * @return bool. true if point is on line segment, false otherwise
         **/
        bool isPointOnSegment(const Eigen::Vector2d &start, const Eigen::Vector2d &end, const Eigen::Vector2d &point);
};