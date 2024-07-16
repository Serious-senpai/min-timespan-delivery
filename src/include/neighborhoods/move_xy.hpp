#pragma once

#include "abc.hpp"

namespace d2d
{
    template <typename ST, int X, int Y>
    class MoveXY : public TabuPairNeighborhood<ST>
    {
        std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> same_route(
            const std::shared_ptr<ST> &solution,
            const std::function<bool(const ST &)> &aspiration_criteria)
        {
            auto problem = Problem::get_instance();
            std::shared_ptr<ST> result;
            std::pair<std::size_t, std::size_t> tabu_pair;

            std::vector<std::vector<TruckRoute>> truck_routes(solution->truck_routes);
            std::vector<std::vector<DroneRoute>> drone_routes(solution->drone_routes);

#define MODIFY_ROUTES(vehicles_count, vehicle_routes, X, Y)                                                                           \
    {                                                                                                                                 \
        for (std::size_t index = 0; index < problem->vehicles_count; index++)                                                         \
        {                                                                                                                             \
            for (std::size_t route = 0; route < vehicle_routes[index].size(); route++)                                                \
            {                                                                                                                         \
                const std::vector<std::size_t> &customers = solution->vehicle_routes[index][route].customers();                       \
                for (std::size_t i = 1; i + 1 < customers.size(); i++)                                                                \
                {                                                                                                                     \
                    for (std::size_t j = i + X; j + Y < customers.size(); j++)                                                        \
                    {                                                                                                                 \
                        /* Temporary modify the route */                                                                              \
                        std::vector<std::size_t> new_customers(customers);                                                            \
                        if constexpr (X > Y)                                                                                          \
                        {                                                                                                             \
                            std::swap_ranges(new_customers.begin() + i, new_customers.begin() + i + Y, new_customers.begin() + j);    \
                            std::rotate(new_customers.begin() + i + Y, new_customers.begin() + i + X, new_customers.begin() + j + Y); \
                        }                                                                                                             \
                        else                                                                                                          \
                        {                                                                                                             \
                            std::swap_ranges(new_customers.begin() + i, new_customers.begin() + i + X, new_customers.begin() + j);    \
                            std::rotate(new_customers.begin() + i + X, new_customers.begin() + j + X, new_customers.begin() + j + Y); \
                        }                                                                                                             \
                                                                                                                                      \
                        using VehicleRoute = std::remove_reference_t<decltype(vehicle_routes[index][route])>;                         \
                        vehicle_routes[index][route] = VehicleRoute(new_customers);                                                   \
                                                                                                                                      \
                        auto new_solution = std::make_shared<ST>(truck_routes, drone_routes);                                         \
                        if ((aspiration_criteria(*new_solution) || !this->is_tabu(customers[i], customers[j])) &&                     \
                            (result == nullptr || new_solution->cost() < result->cost()))                                             \
                        {                                                                                                             \
                            result.swap(new_solution);                                                                                \
                            tabu_pair = std::make_pair(customers[i], customers[j]);                                                   \
                        }                                                                                                             \
                                                                                                                                      \
                        /* Restore */                                                                                                 \
                        vehicle_routes[index][route] = solution->vehicle_routes[index][route];                                        \
                    }                                                                                                                 \
                }                                                                                                                     \
            }                                                                                                                         \
        }                                                                                                                             \
    }

            MODIFY_ROUTES(trucks_count, truck_routes, X, Y);
            MODIFY_ROUTES(drones_count, drone_routes, X, Y);
            if constexpr (X != Y)
            {
                MODIFY_ROUTES(trucks_count, truck_routes, Y, X);
                MODIFY_ROUTES(drones_count, drone_routes, Y, X);
            }

#undef MODIFY_ROUTES

            return std::make_pair(result, tabu_pair);
        }

        std::shared_ptr<ST> move(
            const std::shared_ptr<ST> &solution,
            const std::function<bool(const ST &)> &aspiration_criteria)
        {
            std::shared_ptr<ST> result;
            std::pair<std::size_t, std::size_t> tabu_pair;

            const auto update = [&result, &tabu_pair](const std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> &r)
            {
                if (r.first != nullptr && (result == nullptr || r.first->cost() < result->cost()))
                {
                    result = r.first;
                    tabu_pair = r.second;
                }
            };

            update(same_route(solution, aspiration_criteria));
            this->add_to_tabu(tabu_pair.first, tabu_pair.second);

            return result;
        }
    };

    template <typename ST, int X>
    class MoveXY<ST, X, 0> : public TabuPairNeighborhood<ST>
    {
        std::shared_ptr<ST> move(
            const std::shared_ptr<ST> &solution,
            const std::function<bool(const ST &)> &aspiration_criteria)
        {
        }
    };

    template <typename ST>
    class MoveXY<ST, 0, 0> : public TabuPairNeighborhood<ST>
    {
        std::shared_ptr<ST> move(
            const std::shared_ptr<ST> &solution,
            const std::function<bool(const ST &)> &aspiration_criteria)
        {
            return nullptr;
        }
    };
}
