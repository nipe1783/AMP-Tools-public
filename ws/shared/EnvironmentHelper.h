#pragma once

#include "AMPCore.h"

class EnvironmentHelper {
    public:
        /**
         * @brief Computes distance from robots position pos to a point on the ith obstacle in the environment.
         * 
         * @param prob robot environment.
         * @param pos robots position.
         * @param i index of the obstacle.
         * @return distance to point on obstacle
         **/
        static double distToObstacle(const amp::Problem2D& prob, Eigen::Vector2d pos, int i);

        /**
         * @brief Computes the centroid of the ith obstacle in the environment.
         * 
         * @param prob robot environment
         * @param i index of the obstacle
         * @return Eigen::Vector2d centroid of the ith obstacle
         **/
        static Eigen::Vector2d computeCentroid(amp::Polygon polygon);

        /**
         * @brief returns if two line segments are intersecting.
         * 
         * @param p1 first segment start point
         * @param p2 first segment end point
         * @param p3 second segment start point
         * @param p4 second segment end point
         * @return bool, true if the segments intersect, false otherwise
         **/
        static bool isIntersect(const Eigen::Vector2d &vert1, const Eigen::Vector2d &vert2, const Eigen::Vector2d &vert3, const Eigen::Vector2d &vert4);

        /**
         * @brief Computes the intersection point of two line segments.
         * 
         * @param p1 first segment start point
         * @param p2 first segment end point
         * @param p3 second segment start point
         * @param p4 second segment end point
         * @return Eigen::Vector2d intersection point
         **/
        static Eigen::Vector2d getIntersect(const Eigen::Vector2d &vert1, const Eigen::Vector2d &vert2, const Eigen::Vector2d &vert3, const Eigen::Vector2d &vert4);
};