#include "Helper.h"
#include <Eigen/Geometry> 

bool Helper::isPointOnSegment(const Eigen::Vector2d &start, const Eigen::Vector2d &end, const Eigen::Vector2d &point){
    Eigen::Hyperplane<double, 2> line = Eigen::Hyperplane<double, 2>::Through(start, end);
    double epsilon = 1e-10;
    // Check if the point lies on the line
    if (std::abs(line.signedDistance(point)) > epsilon) {
        return false;
    }
    
    // Check if the point lies within the bounds of the segment
    bool isWithinXBounds = point[0] >= std::min(start[0], end[0]) && point[0] <= std::max(start[0], end[0]);
    float diffX1 = std::fabs(point[0] - start[0]);
    float diffX2 = std::fabs(point[0] - end[0]);
    if (diffX1 < epsilon && diffX2 < epsilon) {
        isWithinXBounds = true;
    }
    bool isWithinYBounds = point[1] >= std::min(start[1], end[1]) && point[1] <= std::max(start[1], end[1]);
    float diffY1 = std::fabs(point[1] - start[1]);
    float diffY2 = std::fabs(point[1] - end[1]);
    if (diffY1 < epsilon && diffY2 < epsilon) {
        isWithinYBounds = true;
    }
    if (isWithinXBounds && isWithinYBounds) {
        return true;
    }
    
    return false;
};

double Helper::distance(Eigen::Vector2d p1, Eigen::Vector2d p2){
    return std::sqrt(std::pow(p1[0] - p2[0], 2) + std::pow(p1[1] - p2[1], 2));
}

Eigen::Vector2d Helper::shortestDist(Eigen::Vector2d vert1, Eigen::Vector2d vert2, Eigen::Vector2d point){
    Eigen::Vector2d dir = vert2 - vert1;
    double lengthSquared = dir.squaredNorm();  // length of vert1-vert2 squared

    // If the segment is just a point, return the point
    if(lengthSquared == 0.0) return vert1;

    // Calculate the projection of point onto the line defined by vert1 and vert2
    double t = (point - vert1).dot(dir) / lengthSquared;

    // If t is in the [0, 1] range, it means the projection is within the segment
    if(t < 0.0) return vert1;
    else if(t > 1.0) return vert2;

    Eigen::Vector2d projection = vert1 + t * dir;
    return projection;
}