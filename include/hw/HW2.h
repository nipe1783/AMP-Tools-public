#pragma once

#include "tools/Environment.h"
#include "tools/Path.h"

namespace amp {

/// @brief Derive this class and implement your algorithm in the `plan` method
class BugAlgorithm {
    public:
        /// @brief Solve a motion planning problem using a bug algorithm. Create a derived class and override this method
        /// @param problem Motion planning problem
        /// @return Path solution
        virtual amp::Path2D plan(const amp::Problem2D& problem) = 0;
};

class HW2 {
    public:
        /// @brief Get WO1 described in Exercise 2.
        /// @return Workspace 1
        static amp::Problem2D getWorkspace1();

        /// @brief Get WO2 described in Exercise 2.
        /// @return Workspace 2
        static amp::Problem2D getWorkspace2();

        /// @brief Checks the path generated by the bug algorithm against the problem
        /// @param path Path generated by bug algorithm
        /// @param prob Problem that path was generated on
        /// @param verbose Output logs displaying result
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool check(const amp::Path2D& path, const amp::Problem2D& prob, bool verbose = true);

        /// @brief Generates a random problem, runs the algorithm, then check the validity of the returned solution.
        /// Similar to what the benchmarker in `grade()` will do
        /// @param algo Your implemented bug algorithm
        /// @param seed Seed the random generator. If the seed is `0u`, no seed is used (random)
        /// @param verbose Output logs displaying result
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool generateAndCheck(const BugAlgorithm& algo, bool verbose = true, uint32_t seed = 0u);

        /// @brief Generates a random problem, runs the algorithm, then check the validity of the returned solution.
        /// Similar to what the benchmarker in `grade()` will do
        /// @param algo Your implemented bug algorithm
        /// @param path Return the path generated by your algorithm
        /// @param prob Return the randomly generated problem used
        /// @param seed Seed the random generator. If the seed is `0u`, no seed is used (random)
        /// @param verbose Output logs displaying result
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool generateAndCheck(const BugAlgorithm& algo, amp::Path2D& path, amp::Problem2D& prob, bool verbose = true, uint32_t seed = 0u);

        // COMING SOON :)
        /// @brief Tests your algorithm on Workspace1, Workspace2, and benchmars your algorithm against many randomized environments
        /// @param algo Your implemented bug algorithm
        /// @return `true`
        //static Result grade(const BugAlgorithm& algo);

};

}