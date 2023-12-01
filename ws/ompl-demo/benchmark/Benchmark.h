# pragma once

#include "AMPCore.h"
#include "../plannerOmpl/PlannerOmpl.h"

class Benchmark{

    public:
        // methods:
        static std::tuple<std::vector<double>, std::vector<double>, std::vector<bool>> runBenchmark(const amp::Problem2D& prob, PlannerOmpl planner, int iterations);
    
};