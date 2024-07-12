#pragma once

#include "initial.hpp"
#include "problem.hpp"
#include "random.hpp"
#include "routes.hpp"
#include "neighborhoods/two_opt.hpp"

namespace d2d
{
    /** @brief Represents a solution to the D2D problem. */
    class Solution
    {
    private:
        static const std::vector<std::shared_ptr<Neighborhood>> neighborhoods;
        static double _calculate_working_time(
            const std::vector<std::vector<TruckRoute>> &truck_routes,
            const std::vector<std::vector<DroneRoute>> &drone_routes);
        static double _calculate_waiting_time_violation(
            const std::vector<std::vector<TruckRoute>> &truck_routes,
            const std::vector<std::vector<DroneRoute>> &drone_routes);
        static double _calculate_energy_violation(const std::vector<std::vector<DroneRoute>> &drone_routes);
        static double _calculate_capacity_violation(
            const std::vector<std::vector<TruckRoute>> &truck_routes,
            const std::vector<std::vector<DroneRoute>> &drone_routes);

    public:
        /** @brief System working time */
        const double working_time;

        /** @brief Total waiting time violation of customers */
        const double waiting_time_violation;

        /** @brief Total drone energy violation */
        const double drone_energy_violation;

        /** @brief Total capacity violation */
        const double capacity_violation;

        /** @brief Routes of trucks */
        const std::vector<std::vector<TruckRoute>> truck_routes;

        /** @brief Routes of drones */
        const std::vector<std::vector<DroneRoute>> drone_routes;

        Solution(
            const std::vector<std::vector<TruckRoute>> &truck_routes,
            const std::vector<std::vector<DroneRoute>> &drone_routes)
            : working_time(_calculate_working_time(truck_routes, drone_routes)),
              waiting_time_violation(_calculate_waiting_time_violation(truck_routes, drone_routes)),
              drone_energy_violation(_calculate_energy_violation(drone_routes)),
              capacity_violation(_calculate_capacity_violation(truck_routes, drone_routes)),
              truck_routes(truck_routes),
              drone_routes(drone_routes)
        {
#ifdef DEBUG
            auto problem = Problem::get_instance();
            std::vector<bool> exists(problem->customers.size());

#define CHECK_ROUTES(vehicle_routes)                                                                             \
    for (auto &routes : vehicle_routes)                                                                          \
    {                                                                                                            \
        for (auto &route : routes)                                                                               \
        {                                                                                                        \
            for (auto &customer : route.customers())                                                             \
            {                                                                                                    \
                if (exists[customer] && customer != 0)                                                           \
                {                                                                                                \
                    throw std::runtime_error(utils::format("Customer %lu is visited more than once", customer)); \
                }                                                                                                \
                                                                                                                 \
                exists[customer] = true;                                                                         \
            }                                                                                                    \
        }                                                                                                        \
    }

            CHECK_ROUTES(truck_routes);
            CHECK_ROUTES(drone_routes);
#undef CHECK_ROUTES

            for (std::size_t i = 0; i < exists.size(); i++)
            {
                if (!exists[i])
                {
                    throw std::runtime_error(utils::format("Missing customer %lu", i));
                }
            }

#endif
        }

        /** @brief Objective function evaluation, including penalties. */
        double cost() const
        {
            return working_time;
        }

        static std::shared_ptr<Solution> initial();
        static std::shared_ptr<Solution> post_optimization(const std::shared_ptr<Solution> &solution);
        static std::shared_ptr<Solution> tabu_search();
    };

    const std::vector<std::shared_ptr<Neighborhood>> Solution::neighborhoods = {
        std::make_shared<TwoOpt>()};

    double Solution::_calculate_working_time(
        const std::vector<std::vector<TruckRoute>> &truck_routes,
        const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        double result = 0;

#define CALCULATE_D2D_ROUTES(vehicle_routes) \
    for (auto &routes : vehicle_routes)      \
    {                                        \
        double time = 0;                     \
        for (auto &route : routes)           \
        {                                    \
            time += route.working_time();    \
        }                                    \
                                             \
        result = std::max(result, time);     \
    }

        CALCULATE_D2D_ROUTES(truck_routes);
        CALCULATE_D2D_ROUTES(drone_routes);
#undef CALCULATE_D2D_ROUTES

        return result;
    }

    double Solution::_calculate_waiting_time_violation(
        const std::vector<std::vector<TruckRoute>> &truck_routes,
        const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        double result = 0;

#define CALCULATE_D2D_ROUTES(vehicle_routes)                 \
    for (auto &routes : vehicle_routes)                      \
    {                                                        \
        for (auto &route : routes)                           \
        {                                                    \
            result += route.waiting_time_violations().sum(); \
        }                                                    \
    }

        CALCULATE_D2D_ROUTES(truck_routes);
        CALCULATE_D2D_ROUTES(drone_routes);
#undef CALCULATE_D2D_ROUTES

        return result;
    }

    double Solution::_calculate_energy_violation(const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        double result = 0;
        for (auto &routes : drone_routes)
        {
            for (auto &route : routes)
            {
                result += route.energy_violation();
            }
        }

        return result;
    }

    double Solution::_calculate_capacity_violation(
        const std::vector<std::vector<TruckRoute>> &truck_routes,
        const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        double result = 0;

#define CALCULATE_D2D_ROUTES(vehicle_routes)      \
    for (auto &routes : vehicle_routes)           \
    {                                             \
        for (auto &route : routes)                \
        {                                         \
            result += route.capacity_violation(); \
        }                                         \
    }

        CALCULATE_D2D_ROUTES(truck_routes);
        CALCULATE_D2D_ROUTES(drone_routes);
#undef CALCULATE_D2D_ROUTES

        return result;
    }

    std::shared_ptr<Solution> Solution::initial()
    {
        return d2d::initial_1();
    }

    std::shared_ptr<Solution> Solution::post_optimization(const std::shared_ptr<Solution> &solution)
    {
        return solution;
    }

    std::shared_ptr<Solution> Solution::tabu_search()
    {
        auto problem = Problem::get_instance();
        auto r = initial(), result = r;

        const auto aspiration_criteria = [&result](const Solution &s)
        {
            return s.cost() < result->cost();
        };

        for (std::size_t iteration = 0; iteration < problem->iterations; iteration++)
        {
            auto neighborhood = utils::random_element(neighborhoods);
            r = neighborhood->move(r, aspiration_criteria);
            if (r->cost() < result->cost())
            {
                result = r;
            }
        }

        return post_optimization(result);
    }
}
