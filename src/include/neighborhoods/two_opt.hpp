#pragma once

#include "abc.hpp"

namespace d2d
{
    class TwoOpt : public Neighborhood
    {
        std::shared_ptr<Solution> move(
            const std::shared_ptr<Solution> &solution,
            const std::function<bool(const Solution &)> &aspiration_criteria) override
        {
        }
    };
}
