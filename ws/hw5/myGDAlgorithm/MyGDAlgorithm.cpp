#include "AMPCore.h"
#include "hw/HW5.h"
#include "MyGDAlgorithm.h"
#include "EnvironmentHelper.h"
#include "Helper.h"

namespace amp{
    MyGDAlgorithm::MyGDAlgorithm() {};
    MyGDAlgorithm::~MyGDAlgorithm() {};

    Path2D MyGDAlgorithm::plan(const Problem2D& prob){
        Path2D path;
        Eigen::Vector2d currPos = prob.q_init;
        Eigen::Vector2d nextPos;
        Eigen::Vector2d dir;
        Eigen::Vector2d att;
        Eigen::Vector2d rep;
        double stepSize = 0.5;
        double dStarGoal = 0.2;
        double zeta = 0.1;
        double nu = 0.005;
        double qStar = 5;
        int i = 0;
        double pathLength = 0;
        path.waypoints.push_back(currPos);
        // currPos = currPos + Eigen::Vector2d(0.0, -0.1);
        while(Helper().distance(currPos, prob.q_goal) > 0.25 && i <5000){
            att = uAtt(prob, currPos, zeta, dStarGoal);
            rep = uRep(prob, currPos, nu, qStar);
            dir = att + rep;
            // std::cout<<"x: "<< -dir[0] << " y: "<< -dir[1] << " i: " << i << std::endl;
            nextPos = currPos - stepSize * (dir);
            pathLength += Helper().distance(currPos, nextPos);
            currPos = nextPos;
            path.waypoints.push_back(currPos);
            i++;
        }
        path.waypoints.push_back(prob.q_goal);
        pathLength += Helper().distance(currPos, prob.q_goal);
        std::cout << "Path Length: " << pathLength << std::endl;
        return path;
    };

    Eigen::Vector2d MyGDAlgorithm::uAtt(const Problem2D& prob, Eigen::Vector2d pos, double zeta, double dStarGoal){
        Eigen::Vector2d step;
        if(Helper().distance(pos, prob.q_goal) <= dStarGoal){
            step = zeta * (pos - prob.q_goal);
        }
        else{
            step = (dStarGoal * zeta) * (pos - prob.q_goal) / Helper().distance(pos, prob.q_goal);
        }
        return step;
    };

    Eigen::Vector2d MyGDAlgorithm::uRep(const Problem2D& prob, Eigen::Vector2d pos, double nu, double qStar){
        Eigen::Vector2d step = Eigen::Vector2d(0, 0);
        Eigen::Vector2d currStep;
        double dist;
        Eigen::Vector2d c;
        for(int i = 0; i < prob.obstacles.size(); i++){
            c = EnvironmentHelper().obstacleRefPoint(prob.obstacles[i], pos);
            dist = Helper().distance(pos, c);
            if(dist < qStar){
                currStep = nu * (1/qStar - 1/(dist)) * ((pos - c) / dist) / std::pow(dist, 2);
                step += currStep;
            }
            else{
                step += Eigen::Vector2d(0, 0);
            }
            c = EnvironmentHelper().computeCentroid(prob.obstacles[i]);
            if(dist < qStar){
                currStep = nu * (1/qStar - 1/(dist)) * ((pos - c) / dist) / std::pow(dist, 2);
                step += currStep;
            }
            else{
                step += Eigen::Vector2d(0, 0);
            }
        };
        return step;
    };

};