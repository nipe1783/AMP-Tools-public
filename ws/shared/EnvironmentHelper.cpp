#include "EnvironmentHelper.h"
#include "Helper.h"
#include <cmath>
#include <Eigen/Geometry>
#include "AMPCore.h"

double EnvironmentHelper::distToObstacle(const amp::Problem2D& prob, Eigen::Vector2d pos, int i){
    double dist = 0;
    return dist;
};

Eigen::Vector2d EnvironmentHelper::computeCentroid(amp::Polygon polygon){
    Eigen::Vector2d centroid(0, 0);
    for (const auto& point : polygon.verticesCCW()) {
        centroid += point;
    }
    return centroid / static_cast<double>(polygon.verticesCCW().size());
};

bool EnvironmentHelper::isIntersect(const Eigen::Vector2d &vert1, const Eigen::Vector2d &vert2, const Eigen::Vector2d &vert3, const Eigen::Vector2d &vert4){
    Eigen::Hyperplane<double,2> plane1 = Eigen::Hyperplane<double,2>::Through(vert1, vert2);
    Eigen::Hyperplane<double,2> plane2 = Eigen::Hyperplane<double,2>::Through(vert3, vert4);
    Eigen::Vector2d intersection = plane1.intersection(plane2);
    if(Helper().isPointOnSegment(vert1, vert2, intersection)){
        return true;
    }
    return false;
};

Eigen::Vector2d EnvironmentHelper::getIntersect(const Eigen::Vector2d &vert1, const Eigen::Vector2d &vert2, const Eigen::Vector2d &vert3, const Eigen::Vector2d &vert4) {
    Eigen::Hyperplane<double,2> plane1 = Eigen::Hyperplane<double,2>::Through(vert1, vert2);
    Eigen::Hyperplane<double,2> plane2 = Eigen::Hyperplane<double,2>::Through(vert3, vert4);
    Eigen::Vector2d intersection = plane1.intersection(plane2);
    return intersection;
};
