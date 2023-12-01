#include "AMPCore.h"
#include "hw/HW7.h"
#include "PlannerOmpl.h"
#include "GradePlanner.h"

namespace amp{

    Path2D GradePlanner::plan(const Problem2D& prob){
    
        PlannerOmpl planner;
        amp::Path2D path = planner.planGeometric(prob);
        if(path.waypoints.size() == 0){
            path.waypoints.push_back(prob.q_init);
            path.waypoints.push_back(prob.q_goal);
        }
        Visualizer::makeFigure(prob, path);
        Visualizer::showFigures();
        return path;
    }

}