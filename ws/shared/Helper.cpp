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