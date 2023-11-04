#include "EnvironmentHelper.h"
#include "Helper.h"
#include <cmath>
#include <Eigen/Geometry>
#include "AMPCore.h"
#include "MyConfigurationSpace.h"

 bool EnvironmentHelper::collisionNRobot(const amp::MultiAgentProblem2D& problem, const Eigen::VectorXd& q1, const Eigen::VectorXd& q2){
    // check if any robot collides with obstacle
    for(int i = 0; i < problem.numAgents(); i++){
        for(amp::Obstacle2D obstacle : problem.obstacles){
            Eigen::Vector2d p1(q1[i*2], q1[i*2+1]);
            Eigen::Vector2d p2(q2[i*2], q2[i*2+1]);
            if(Helper().intersects(problem, p1, p2)){
                return true;
            }
        }
    }
    // check if any robot collides with another robot
    // TODO
    return false;
 }

Eigen::Vector2d EnvironmentHelper::obstacleRefPoint(const amp::Obstacle2D &obstacle, Eigen::Vector2d &pos){
    Eigen::Vector2d refPoint;
    // step 1: compute centroid
    Eigen::Vector2d centroid = computeCentroid(obstacle);

    // step 2: check if pos to centroid intersect.
    for(int i = 0; i < obstacle.verticesCCW().size(); i++){
        if(i == obstacle.verticesCCW().size() - 1){
            if(isIntersect(pos, centroid, obstacle.verticesCCW()[i], obstacle.verticesCCW()[0])){
                // step 3: return intersect point
                refPoint = Helper().shortestDist(obstacle.verticesCCW()[i], obstacle.verticesCCW()[0], pos);
            }
        }
        else{
            if(isIntersect(pos, centroid, obstacle.verticesCCW()[i], obstacle.verticesCCW()[i+1])){
                // step 3: return intersect point
                refPoint = Helper().shortestDist(obstacle.verticesCCW()[i], obstacle.verticesCCW()[i+1], pos);
            }
        }
    }
    return refPoint;
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

 std::unique_ptr<amp::GridCSpace2D> EnvironmentHelper::constructCSpacePRB(const amp::Environment2D& environment, double grid_size, float delta){
        
        double density_x0 = (environment.x_max - environment.x_min) / grid_size;
        double density_x1 = (environment.y_max - environment.y_min) / grid_size;

        std::unique_ptr<amp::GridCSpace2D> cSpace = std::make_unique<amp::MyConfigurationSpace>(density_x0, density_x1, environment.x_min, environment.x_max, environment.y_min, environment.y_max);
        Eigen::Vector2d point;
        amp::Obstacle2D expandedObstacle;
        std::pair<std::size_t, std::size_t> cell;
        for(double i = environment.x_min; i < environment.x_max; i += grid_size){
            for(double j = environment.y_min; j < environment.y_max; j += grid_size){
                point = {i, j};
                for(amp::Obstacle2D obstacle : environment.obstacles){
                    expandedObstacle = Helper().expandObstacle(obstacle, delta);
                    if(Helper().inCollision(point, expandedObstacle)){
                        cell = cSpace->getCellFromPoint(i, j);
                        cSpace->operator()(cell.first, cell.second) = true;
                    }
                }
            }
        }

        return cSpace;
 };

bool EnvironmentHelper::inCollision(const Eigen::Vector2d& point, const amp::Obstacle2D& obstacle) {
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