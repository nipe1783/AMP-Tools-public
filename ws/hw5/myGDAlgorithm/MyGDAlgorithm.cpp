#include "AMPCore.h"
#include "hw/HW5.h"
#include "MyGDAlgorithm.h"
#include "EnvironmentHelper.h"

namespace amp{
    MyGDAlgorithm::MyGDAlgorithm() {};
    MyGDAlgorithm::~MyGDAlgorithm() {};

    Path2D MyGDAlgorithm::plan(const Problem2D& prob){
        Path2D path;
        Eigen::Vector2d currPos = prob.q_init;
        Eigen::Vector2d nextPos;
        double stepSize = 0.1;
        // path.waypoints.push_back(prob.q_init);
        // path.waypoints.push_back(prob.q_goal);
        int i = 0;
        while(i < 10){
            nextPos = currPos - stepSize * (uAtt(prob) + uRep(prob));
        }
        // Eigen::Vector2d centroid = EnvironmentHelper().computeCentroid(prob.obstacles[0]);
        // std::cout<<centroid<<std::endl;
        return path;
    };

    Eigen::Vector2d MyGDAlgorithm::uAtt(const Problem2D& prob){
        Eigen::Vector2d step;
        return step;
    };

};