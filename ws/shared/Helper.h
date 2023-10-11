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

        /**
         * @brief Computes distance from p1 to p2
         * 
         * @param p1 Start Point
         * @param p2 End Point
         * @return double, distance between p1 and p2
         **/
        double distance(Eigen::Vector2d p1, Eigen::Vector2d p2);
        
        Eigen::Vector2d shortestDist(Eigen::Vector2d vert1, Eigen::Vector2d vert2, Eigen::Vector2d point);
};