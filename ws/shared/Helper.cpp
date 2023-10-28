#include "Helper.h"
#include "EnvironmentHelper.h"
#include <Eigen/Geometry> 
#include <iostream>
#include <vector>
#include <numeric>

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

amp::Polygon Helper::minkowskiSum(const amp::Polygon& p1,const amp::Polygon& p2){

    float angle1;
    float angle2;
    int i = 0;
    int j = 0;
    int n = p1.verticesCCW().size();
    int m = p2.verticesCCW().size();
    Eigen::Vector2d newVert;
    amp::Polygon cObstacle;
    while( i != n + 1 && j != m + 1){
        if(i == n && j == m){
            newVert = p1.verticesCCW()[0] + p2.verticesCCW()[0];
        }
        else if (i == n)
        {
            newVert = p1.verticesCCW()[0] + p2.verticesCCW()[j];
        }
        else if (j == m)
        {
            newVert = p1.verticesCCW()[i] + p2.verticesCCW()[0];
        }
        else{
            newVert = p1.verticesCCW()[i] + p2.verticesCCW()[j];
        }
        cObstacle.verticesCCW().push_back(newVert);
        if(i == n-1){
            angle1 = Helper().angle(p1.verticesCCW()[i], p1.verticesCCW()[0]);
        }
        else{
            angle1 = Helper().angle(p1.verticesCCW()[i], p1.verticesCCW()[i+1]);
        }
        if(j == m-1){
            angle2 = Helper().angle(p2.verticesCCW()[j], p2.verticesCCW()[0]);
        }
        else{
            angle2 = Helper().angle(p2.verticesCCW()[j], p2.verticesCCW()[j+1]);
        }
        if(angle1 < angle2){
            i++;
        }
        else if (angle1 > angle2)
        {
            j++;
        }
        else{   
            i++;
            j++;
        }
    }
    return cObstacle;
}

/**
 * @brief Returns true if line segment intersects with any obstacle in environment.
 * 
 * @param environment Description of parameter
 * @return true for interesection false for no intersection.
 **/
bool Helper::intersects(const amp::Environment2D& environment, Eigen::Vector2d& start, Eigen::Vector2d& end){
    Eigen::Hyperplane<double,2> plane1;
    Eigen::Hyperplane<double,2> plane2;
    Eigen::Vector2d vert1;
    Eigen::Vector2d vert2;
    Eigen::Vector2d intersection;
    amp::Obstacle2D expandedObstacle;
    plane1 = Eigen::Hyperplane<double,2>::Through(start, end);
    for(amp::Obstacle2D obstacle : environment.obstacles){
        std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
        for(size_t i = 0; i < vertices.size(); i++){
            if (i == vertices.size() - 1) {
                // create line from two vertices.
                vert1 = vertices[0];
                vert2 = vertices[i];
                plane2 = Eigen::Hyperplane<double,2>::Through(vert1, vert2);
                intersection = plane1.intersection(plane2);

                //check to see if intersection is on both line segments.
                // case 1: new position is in obstacle.
                if(isPointOnSegment(start, end, intersection) && isPointOnSegment(vert1, vert2, intersection)){
                   return true;
                }
                
            }
            else{
                // create line from two vertices.
                vert1 = vertices[i];
                vert2 = vertices[i+1];
                plane2 = Eigen::Hyperplane<double,2>::Through(vert1, vert2);
                intersection = plane1.intersection(plane2);
                
                //check to see if intersection is on both line segments.
                // case 1: new position is in obstacle.
                if(isPointOnSegment(start, end, intersection) && isPointOnSegment(vert1, vert2, intersection)){
                    return true;
                }
            }
        }
    }
    return false;
}

amp::Polygon Helper::minkowskiDiff(const amp::Polygon& obstacle,const amp::Polygon& robot){
    // Step 1: Negate robot around reference point.
    Eigen::Vector2d refPoint = robot.verticesCCW()[0];
    amp::Polygon negatedRobot;
    amp::Polygon orderedNegatedRobot;
    amp::Polygon orderedObstacle;
    for(int i = 0; i < robot.verticesCCW().size(); i++){
        negatedRobot.verticesCCW().push_back(-robot.verticesCCW()[i]);
    }
    
    // step 2: order vertices so first one is least y value in ccw order.
    orderedNegatedRobot = Helper().orderVertices(negatedRobot);
    orderedObstacle = Helper().orderVertices(obstacle);

    // Step 3: Compute Minkowski sum of obstacle and negated robot.
    amp::Polygon cSpace = minkowskiSum(orderedObstacle, orderedNegatedRobot);
    return cSpace;
}

/**
 * @brief returns angle between two vertices and x axis
 * 
 * @param paramName two vertices
 * @return angle in radians
 **/
float Helper::angle(const Eigen::Vector2d& start,const Eigen::Vector2d& end){
    Eigen::Vector2d direction = end - start;  // Direction vector of the line segment
    float angle = atan2(direction(1), direction(0));
    if (angle < 0) {
        angle += 2 * M_PI;  // Adjust the angle to be in the range [0, 2*pi]
    }
    return angle;
}

/**
 * @brief Orders polygon vertices in with first one being least y value in ccw order.
 * 
 * @param paramName polygon
 * @return ordered polygon
 **/
amp::Polygon Helper::orderVertices(const amp::Polygon& polygon){
    amp::Polygon orderedPolygon;
    auto minElement = std::min_element(polygon.verticesCCW().begin(), polygon.verticesCCW().end(),
        [](const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
            return v1(1) < v2(1);  // Compare based on y-values
        }
    );

    for (auto it = minElement; it != polygon.verticesCCW().end(); ++it) {
        orderedPolygon.verticesCCW().push_back(*it);
    }
    for (auto it = polygon.verticesCCW().begin(); it != minElement; ++it) {
        orderedPolygon.verticesCCW().push_back(*it);
    }
    return orderedPolygon;
}

/**
 * @brief rotates a polygon arount a point by theta radians. creates a new polygon.
 * 
 * @param paramName polygon and theta value
 * @return returnType Description of return value
 **/
amp::Polygon Helper::rotatePolygon(const amp::Polygon& p, Eigen::Vector2d rotationVertex, float theta) {
    amp::Polygon rotatedPolygon;
    Eigen::Matrix2d rotationMatrix;
    rotationMatrix << cos(theta), -sin(theta), sin(theta), cos(theta);

    for (int i = 0; i < p.verticesCCW().size(); i++) {
        // 1. Translate vertex so that rotationVertex becomes the origin
        Eigen::Vector2d translatedVertex = p.verticesCCW()[i] - rotationVertex;
        
        // 2. Rotate the translated vertex
        Eigen::Vector2d rotatedTranslatedVertex = rotationMatrix * translatedVertex;
        
        // 3. Translate the vertex back
        Eigen::Vector2d finalVertex = rotatedTranslatedVertex + rotationVertex;
        
        rotatedPolygon.verticesCCW().push_back(finalVertex);
    }

    return rotatedPolygon;
}

/**
 * @brief given two polygons return true if they intersect.
 **/
bool Helper::polygonIntersect(const amp::Polygon& p1, const amp::Polygon& p2){

    double xmin = std::numeric_limits<double>::infinity();
    double xmax = -std::numeric_limits<double>::infinity();
    double ymin = std::numeric_limits<double>::infinity();
    double ymax = -std::numeric_limits<double>::infinity();
    double x;
    double y;

    // step 1: compute minkowski diff of two polygons
    amp::Polygon cSpace = minkowskiDiff(p1, p2);

    // step 2: determine if origin is in cSpace
    for (int i = 0; i < cSpace.verticesCCW().size(); i++) {
        x = cSpace.verticesCCW()[i][0];
        y = cSpace.verticesCCW()[i][1];
        if (x < xmin) {
            xmin = x;
        }
        if (x > xmax) {
            xmax = x;
        }
        if (y < ymin) {
            ymin = y;
        }
        if (y > ymax) {
            ymax = y;
        }
    }
    if (xmin <= 0 && xmax >= 0 && ymin <= 0 && ymax >= 0) {
        return true;
    }
    return false;
}

bool Helper::inCollision(const Eigen::Vector2d& point, const amp::Obstacle2D& obstacle) {
    bool isInside = false;
    int numVertices = obstacle.verticesCCW().size();

    for (int i = 0, j = numVertices - 1; i < numVertices; j = i++) {
        Eigen::Vector2d vertex1 = obstacle.verticesCCW()[i];
        Eigen::Vector2d vertex2 = obstacle.verticesCCW()[j];

        if (((vertex1[1] > point[1]) != (vertex2[1] > point[1])) &&
            (point[0] < (vertex2[0] - vertex1[0]) * (point[1] - vertex1[1]) / (vertex2[1] - vertex1[1]) + vertex1[0])) {
            isInside = !isInside;
        }
    }

    return isInside;
}


amp::Obstacle2D Helper::expandObstacle(amp::Obstacle2D obstacle, float delta) const {
    amp::Obstacle2D enlargedObstacle;
    std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
    Eigen::Vector2d centroid = EnvironmentHelper().computeCentroid(vertices);

    for (const Eigen::Vector2d& vertex : vertices) {
        Eigen::Vector2d direction = (vertex - centroid).normalized(); // Get the direction from centroid to the vertex and normalize it
        Eigen::Vector2d enlargedVertex = vertex + direction * delta;  // Expand vertex along the direction by delta
        enlargedObstacle.verticesCCW().push_back(enlargedVertex);
    }
    return enlargedObstacle;
}


double Helper::average(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    return sum / v.size();
}