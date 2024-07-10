#pragma once

#include "problem.hpp"
#include "routes.hpp"

namespace d2d
{
    class Solution
    {
    private:
    public:
        const std::vector<TruckRoute> truck_routes;
        const std::vector<std::vector<DroneRoute>> drone_routes;

        Solution(
            const std::vector<TruckRoute> &truck_routes,
            const std::vector<std::vector<DroneRoute>> &drone_routes)
            : truck_routes(truck_routes),
              drone_routes(drone_routes) {}
    };
}
