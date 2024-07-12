#pragma once

#include "../standard.hpp"

namespace d2d
{
    class Solution; // forward declaration

    /**
     * @brief Base class for local search neighborhoods
     */
    class Neighborhood
    {
    public:
        /**
         * @brief Perform a local search to find the best solution in the neighborhood.
         *
         * @param solution A shared pointer to the current solution
         * @param aspiration_criteria The aspiration criteria of tabu search
         * @return The best solution found that is not `solution`
         */
        virtual std::shared_ptr<Solution> move(
            const std::shared_ptr<Solution> &solution,
            const std::function<bool(const Solution &)> &aspiration_criteria) = 0;
    };
}
