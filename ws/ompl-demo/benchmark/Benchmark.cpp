#include "Benchmark.h"

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<bool>> Benchmark::runBenchmark(const amp::Problem2D& prob, PlannerOmpl planner, int iterations){
    
    std::vector<double> times;
    std::vector<double> numNodes;
    std::vector<double> pathLengths;
    std::vector<bool> success;

    for(int i = 0; i < iterations; i++) {
        auto answer = planner.planGeometric(prob, true);
        times.push_back(std::get<1>(answer));
        pathLengths.push_back(std::get<2>(answer));
        numNodes.push_back(std::get<3>(answer));
        success.push_back(std::get<0>(answer).waypoints.size() > 0);
    }

    return std::make_tuple(times, pathLengths, numNodes, success);

}