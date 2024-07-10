#pragma once

#include "problem.hpp"
#include "routes.hpp"

namespace d2d
{
    /** @brief Represents a solution to the D2D problem. */
    class Solution
    {
    private:
        static double _calculate_working_time(
            const std::vector<std::vector<TruckRoute>> &truck_routes,
            const std::vector<std::vector<DroneRoute>> &drone_routes);

    public:
        /** @brief System working time */
        const double working_time;

        /** @brief Routes of trucks */
        const std::vector<std::vector<TruckRoute>> truck_routes;

        /** @brief Routes of drones */
        const std::vector<std::vector<DroneRoute>> drone_routes;

        Solution(
            const std::vector<std::vector<TruckRoute>> &truck_routes,
            const std::vector<std::vector<DroneRoute>> &drone_routes)
            : working_time(_calculate_working_time(truck_routes, drone_routes)),
              truck_routes(truck_routes),
              drone_routes(drone_routes) {}

        /** @brief Objective function evaluation, including penalties. */
        double cost() const
        {
            return working_time;
        }
    };

    double _calculate_working_time(
        const std::vector<std::vector<TruckRoute>> &truck_routes,
        const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        double result = 0;

        for (auto &routes : truck_routes)
        {
            double time = 0;
            for (auto &route : routes)
            {
                time += route.working_time;
            }

            result = std::max(result, time);
        }

        for (auto &routes : drone_routes)
        {
            double time = 0;
            for (auto &route : routes)
            {
                time += route.working_time;
            }

            result = std::max(result, time);
        }

        return result;
    }

    bool operator<(const Solution &first, const Solution &second)
    {
        return first.cost() < second.cost();
    }
}
