#pragma once

#include "abc.hpp"

namespace d2d
{
    template <typename ST>
    class TwoOpt : public TabuPairNeighborhood<ST>
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

#define MODIFY_ROUTES(vehicles_count, vehicle_routes)                                                                 \
    {                                                                                                                 \
        for (std::size_t index = 0; index < problem->vehicles_count; index++)                                         \
        {                                                                                                             \
            for (std::size_t route = 0; route < vehicle_routes[index].size(); route++)                                \
            {                                                                                                         \
                const std::vector<std::size_t> &customers = solution->vehicle_routes[index][route].customers();       \
                for (std::size_t i = 1; i + 1 < customers.size(); i++)                                                \
                {                                                                                                     \
                    for (std::size_t j = i + 1; j + 1 < customers.size(); j++)                                        \
                    {                                                                                                 \
                        /* Temporary reverse segment [i, j] */                                                        \
                        vehicle_routes[index][route].reverse(i, j - i + 1);                                           \
                                                                                                                      \
                        auto new_solution = std::make_shared<ST>(truck_routes, drone_routes);                         \
                        if ((aspiration_criteria(*new_solution) || !this->is_tabu(customers[i - 1], customers[j])) && \
                            (result == nullptr || new_solution->cost() < result->cost()))                             \
                        {                                                                                             \
                            result.swap(new_solution);                                                                \
                            tabu_pair = std::make_pair(customers[i - 1], customers[j]);                               \
                        }                                                                                             \
                                                                                                                      \
                        /* Restore */                                                                                 \
                        vehicle_routes[index][route].reverse(i, j - i + 1);                                           \
                    }                                                                                                 \
                }                                                                                                     \
            }                                                                                                         \
        }                                                                                                             \
    }

            MODIFY_ROUTES(trucks_count, truck_routes);
            MODIFY_ROUTES(drones_count, drone_routes);

#undef MODIFY_ROUTES

            return std::make_pair(result, tabu_pair);
        }

        std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> multi_route(
            const std::shared_ptr<ST> &solution,
            const std::function<bool(const ST &)> &aspiration_criteria)
        {
            auto problem = Problem::get_instance();
            std::shared_ptr<ST> result;
            std::pair<std::size_t, std::size_t> tabu_pair;

            std::vector<std::vector<TruckRoute>> truck_routes(solution->truck_routes);
            std::vector<std::vector<DroneRoute>> drone_routes(solution->drone_routes);
            for (std::size_t vehicle_i = 0; vehicle_i < problem->trucks_count + problem->drones_count; vehicle_i++)
            {
                for (std::size_t vehicle_j = vehicle_i; vehicle_j < problem->trucks_count + problem->drones_count; vehicle_j++)
                {
#define MODIFY_ROUTES(vehicle_routes_i, vehicle_routes_j, vehicle_i, vehicle_j)                                                                                \
    {                                                                                                                                                          \
        for (std::size_t route_i = 0; route_i < solution->vehicle_routes_i[vehicle_i].size(); route_i++)                                                       \
        {                                                                                                                                                      \
            for (std::size_t route_j = 0; route_j < solution->vehicle_routes_j[vehicle_j].size(); route_j++)                                                   \
            {                                                                                                                                                  \
                const std::vector<std::size_t> &customers_i = solution->vehicle_routes_i[vehicle_i][route_i].customers();                                      \
                const std::vector<std::size_t> &customers_j = solution->vehicle_routes_j[vehicle_j][route_j].customers();                                      \
                for (std::size_t i = 0; i + 2 < customers_i.size(); i++)                                                                                       \
                {                                                                                                                                              \
                    for (std::size_t j = 0; j + 2 < customers_j.size(); j++)                                                                                   \
                    {                                                                                                                                          \
                        using VehicleRoute_i = std::remove_reference_t<decltype(vehicle_routes_i[vehicle_i][route_i])>;                                        \
                        using VehicleRoute_j = std::remove_reference_t<decltype(vehicle_routes_j[vehicle_j][route_j])>;                                        \
                        if constexpr (std::is_same_v<VehicleRoute_i, VehicleRoute_j>)                                                                          \
                        {                                                                                                                                      \
                            if (vehicle_i == vehicle_j && route_i == route_j)                                                                                  \
                            {                                                                                                                                  \
                                continue;                                                                                                                      \
                            }                                                                                                                                  \
                        }                                                                                                                                      \
                                                                                                                                                               \
                        std::vector<std::size_t> ri(customers_i.begin(), customers_i.begin() + (i + 1)),                                                       \
                            rj(customers_j.begin(), customers_j.begin() + (j + 1));                                                                            \
                                                                                                                                                               \
                        ri.insert(ri.end(), customers_j.begin() + (j + 1), customers_j.end());                                                                 \
                        rj.insert(rj.end(), customers_i.begin() + (i + 1), customers_i.end());                                                                 \
                                                                                                                                                               \
                        if constexpr (std::is_same_v<VehicleRoute_i, DroneRoute>)                                                                              \
                        {                                                                                                                                      \
                            if (std::any_of(ri.begin(), ri.end(), [&problem](const std::size_t &c) { return !problem->customers[c].dronable; }))               \
                            {                                                                                                                                  \
                                continue;                                                                                                                      \
                            }                                                                                                                                  \
                        }                                                                                                                                      \
                                                                                                                                                               \
                        if constexpr (std::is_same_v<VehicleRoute_j, DroneRoute>)                                                                              \
                        {                                                                                                                                      \
                            if (std::any_of(rj.begin(), rj.end(), [&problem](const std::size_t &c) { return !problem->customers[c].dronable; }))               \
                            {                                                                                                                                  \
                                continue;                                                                                                                      \
                            }                                                                                                                                  \
                        }                                                                                                                                      \
                                                                                                                                                               \
                        bool ri_empty = (ri.size() == 2), rj_empty = (rj.size() == 2);                                                                         \
                        if (ri_empty)                                                                                                                          \
                        {                                                                                                                                      \
                            vehicle_routes_i[vehicle_i].erase(vehicle_routes_i[vehicle_i].begin() + route_i);                                                  \
                        }                                                                                                                                      \
                        else                                                                                                                                   \
                        {                                                                                                                                      \
                            vehicle_routes_i[vehicle_i][route_i] = VehicleRoute_i(ri);                                                                         \
                        }                                                                                                                                      \
                                                                                                                                                               \
                        if (rj_empty)                                                                                                                          \
                        {                                                                                                                                      \
                            vehicle_routes_j[vehicle_j].erase(vehicle_routes_j[vehicle_j].begin() + route_j);                                                  \
                        }                                                                                                                                      \
                        else                                                                                                                                   \
                        {                                                                                                                                      \
                            vehicle_routes_j[vehicle_j][route_j] = VehicleRoute_j(rj);                                                                         \
                        }                                                                                                                                      \
                                                                                                                                                               \
                        auto new_solution = std::make_shared<ST>(truck_routes, drone_routes);                                                                  \
                        if ((aspiration_criteria(*new_solution) || !this->is_tabu(customers_i[i], customers_j[j])) &&                                          \
                            (result == nullptr || new_solution->cost() < result->cost()))                                                                      \
                        {                                                                                                                                      \
                            result.swap(new_solution);                                                                                                         \
                            tabu_pair = std::make_pair(customers_i[i], customers_j[j]);                                                                        \
                        }                                                                                                                                      \
                                                                                                                                                               \
                        /* Restore */                                                                                                                          \
                        if (ri_empty)                                                                                                                          \
                        {                                                                                                                                      \
                            vehicle_routes_i[vehicle_i].insert(vehicle_routes_i[vehicle_i].begin() + route_i, solution->vehicle_routes_i[vehicle_i][route_i]); \
                        }                                                                                                                                      \
                        else                                                                                                                                   \
                        {                                                                                                                                      \
                            vehicle_routes_i[vehicle_i][route_i] = solution->vehicle_routes_i[vehicle_i][route_i];                                             \
                        }                                                                                                                                      \
                                                                                                                                                               \
                        if (rj_empty)                                                                                                                          \
                        {                                                                                                                                      \
                            vehicle_routes_j[vehicle_j].insert(vehicle_routes_j[vehicle_j].begin() + route_j, solution->vehicle_routes_j[vehicle_j][route_j]); \
                        }                                                                                                                                      \
                        else                                                                                                                                   \
                        {                                                                                                                                      \
                            vehicle_routes_j[vehicle_j][route_j] = solution->vehicle_routes_j[vehicle_j][route_j];                                             \
                        }                                                                                                                                      \
                    }                                                                                                                                          \
                }                                                                                                                                              \
            }                                                                                                                                                  \
        }                                                                                                                                                      \
    }

                    if (vehicle_i < problem->trucks_count)
                    {
                        if (vehicle_j < problem->trucks_count)
                        {
                            MODIFY_ROUTES(truck_routes, truck_routes, vehicle_i, vehicle_j);
                        }
                        else
                        {
                            MODIFY_ROUTES(truck_routes, drone_routes, vehicle_i, vehicle_j - problem->trucks_count);
                        }
                    }
                    else
                    {
                        MODIFY_ROUTES(drone_routes, drone_routes, vehicle_i - problem->trucks_count, vehicle_j - problem->trucks_count);
                    }

#undef MODIFY_ROUTES
                }
            }

            return std::make_pair(result, tabu_pair);
        }

        std::shared_ptr<ST> move(
            const std::shared_ptr<ST> &solution,
            const std::function<bool(const ST &)> &aspiration_criteria) override
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
            update(multi_route(solution, aspiration_criteria));

            this->add_to_tabu(tabu_pair.first, tabu_pair.second);

            return result;
        }
    };
}
