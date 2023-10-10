#include "AMPCore.h"
#include "hw/HW5.h"
#include "MyGDAlgorithm.h"

namespace amp{
    MyGDAlgorithm::MyGDAlgorithm() {}
    MyGDAlgorithm::~MyGDAlgorithm() {}

    Path2D MyGDAlgorithm::plan(const Problem2D& prob){
        Path2D path;
        path.waypoints.push_back(prob.q_init);
        path.waypoints.push_back(prob.q_goal);
        return path;
    };
};