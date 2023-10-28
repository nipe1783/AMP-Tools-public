#pragma once

#include "AMPCore.h"

class EnvironmentHelper {
    public:
        /**
         * @brief Computes closest point on obstacles perimeter to robots position pos.
         * 
         * @param prob robot environment.
         * @param pos robots position.
         * @param i index of the obstacle.
         * @return obstacle reference point
         **/
        static Eigen::Vector2d obstacleRefPoint(const amp::Obstacle2D &obstacle, Eigen::Vector2d &pos);

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


        /**
         * @brief Creates discretized cspace for point robot.
         * 
         * @param environment robot environment
         * @param grid_size step size in cspace
         * @param delta padding on obstacles
         * @return returnType Description of return value
         **/
        std::unique_ptr<amp::GridCSpace2D> constructCSpacePRB(const amp::Environment2D& environment, double grid_size, float delta);

        /**
         * @brief Returns true if point is in obstacle.
         * 
         * @param point point to check
         * @return true if point is in obstacle
         **/
        bool inCollision(const Eigen::Vector2d& point, const amp::Obstacle2D& obstacle);
        
};