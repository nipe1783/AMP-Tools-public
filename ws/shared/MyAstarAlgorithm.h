#pragma once
#include "AMPCore.h"
#include "hw/HW6.h"
#include "Tree.h"
#include "MyGridCSpace2DConstructor.h"

namespace amp {
    class MyAStarAlgorithm : public amp::AStar {
        public:
            GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
    };
}
