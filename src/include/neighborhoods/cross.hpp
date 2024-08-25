#pragma once

#include "abc.hpp"

namespace d2d
{
    template <typename ST>
    class CrossExchange : public Neighborhood<ST, false>
    {
    public:
        std::string label() const override
        {
            return "CROSS-exchange";
        }

        std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> intra_route(
            std::shared_ptr<ST> solution,
            const std::function<bool(std::shared_ptr<ST>)> &aspiration_criteria) override
        {
            return std::make_pair(nullptr, std::make_pair(0, 0));
        }

        std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> inter_route(
            std::shared_ptr<ST> solution,
            const std::function<bool(std::shared_ptr<ST>)> &aspiration_criteria) override
        {
            auto problem = Problem::get_instance();
            auto parent = this->parent_ptr(solution);
            std::shared_ptr<ST> result;

            std::vector<std::vector<TruckRoute>> truck_routes(solution->truck_routes);
            std::vector<std::vector<DroneRoute>> drone_routes(solution->drone_routes);

            for (std::size_t vehicle_i = 0; vehicle_i < problem->trucks_count + problem->drones_count; vehicle_i++)
            {
                for (std::size_t vehicle_j = vehicle_i; vehicle_j < problem->trucks_count + problem->drones_count; vehicle_j++)
                {
#define MODIFY_ROUTES(vehicle_routes_i, vehicle_routes_j)                                                                                                                 \
    {                                                                                                                                                                     \
        std::size_t _vehicle_i = vehicle_i < problem->trucks_count ? vehicle_i : vehicle_i - problem->trucks_count,                                                       \
                    _vehicle_j = vehicle_j < problem->trucks_count ? vehicle_j : vehicle_j - problem->trucks_count;                                                       \
        for (std::size_t route_i = 0; route_i < solution->vehicle_routes_i[_vehicle_i].size(); route_i++)                                                                 \
        {                                                                                                                                                                 \
            for (std::size_t route_j = 0; route_j < solution->vehicle_routes_j[_vehicle_j].size(); route_j++)                                                             \
            {                                                                                                                                                             \
                using VehicleRoute_i = std::remove_cvref_t<decltype(vehicle_routes_i[_vehicle_i][route_i])>;                                                              \
                using VehicleRoute_j = std::remove_cvref_t<decltype(vehicle_routes_j[_vehicle_j][route_j])>;                                                              \
                                                                                                                                                                          \
                if constexpr (std::is_same_v<VehicleRoute_i, VehicleRoute_j>)                                                                                             \
                {                                                                                                                                                         \
                    if (_vehicle_i == _vehicle_j && route_i == route_j) /* same route */                                                                                  \
                    {                                                                                                                                                     \
                        continue;                                                                                                                                         \
                    }                                                                                                                                                     \
                }                                                                                                                                                         \
                                                                                                                                                                          \
                const std::vector<std::size_t> &customers_i = solution->vehicle_routes_i[_vehicle_i][route_i].customers();                                                \
                const std::vector<std::size_t> &customers_j = solution->vehicle_routes_j[_vehicle_j][route_j].customers();                                                \
                                                                                                                                                                          \
                for (std::size_t i = 1; i + 1 < customers_i.size(); i++)                                                                                                  \
                {                                                                                                                                                         \
                    for (std::size_t j = 1; j + 1 < customers_j.size(); j++)                                                                                              \
                    {                                                                                                                                                     \
                        for (std::size_t ix = i; ix < customers_i.size(); ix++)                                                                                           \
                        {                                                                                                                                                 \
                            for (std::size_t jx = j; jx < customers_j.size(); jx++)                                                                                       \
                            {                                                                                                                                             \
                                if constexpr (std::is_same_v<VehicleRoute_i, DroneRoute> && std::is_same_v<VehicleRoute_j, TruckRoute>)                                   \
                                {                                                                                                                                         \
                                    if (std::any_of(                                                                                                                      \
                                            customers_j.begin() + j, customers_j.begin() + jx,                                                                            \
                                            [&problem](const std::size_t &c) { return !problem->customers[c].dronable; }))                                                \
                                    {                                                                                                                                     \
                                        continue;                                                                                                                         \
                                    }                                                                                                                                     \
                                }                                                                                                                                         \
                                                                                                                                                                          \
                                if constexpr (std::is_same_v<VehicleRoute_i, TruckRoute> && std::is_same_v<VehicleRoute_j, DroneRoute>)                                   \
                                {                                                                                                                                         \
                                    if (std::any_of(                                                                                                                      \
                                            customers_i.begin() + i, customers_i.begin() + ix,                                                                            \
                                            [&problem](const std::size_t &c) { return !problem->customers[c].dronable; }))                                                \
                                    {                                                                                                                                     \
                                        continue;                                                                                                                         \
                                    }                                                                                                                                     \
                                }                                                                                                                                         \
                                                                                                                                                                          \
                                std::vector<std::size_t> ri(customers_i.begin(), customers_i.begin() + i),                                                                \
                                    rj(customers_j.begin(), customers_j.begin() + j);                                                                                     \
                                                                                                                                                                          \
                                ri.insert(ri.end(), customers_j.begin() + j, customers_j.begin() + jx);                                                                   \
                                rj.insert(rj.end(), customers_i.begin() + i, customers_i.begin() + ix);                                                                   \
                                                                                                                                                                          \
                                ri.insert(ri.end(), customers_i.begin() + ix, customers_i.end());                                                                         \
                                rj.insert(rj.end(), customers_j.begin() + jx, customers_j.end());                                                                         \
                                                                                                                                                                          \
                                bool ri_empty = (ri.size() == 2), rj_empty = (rj.size() == 2); /* Note: These values can't be true at the same time */                    \
                                if (ri_empty)                                                                                                                             \
                                {                                                                                                                                         \
                                    vehicle_routes_j[_vehicle_j][route_j] = VehicleRoute_j(rj);                                                                           \
                                    vehicle_routes_i[_vehicle_i].erase(vehicle_routes_i[_vehicle_i].begin() + route_i);                                                   \
                                }                                                                                                                                         \
                                else if (rj_empty)                                                                                                                        \
                                {                                                                                                                                         \
                                    vehicle_routes_i[_vehicle_i][route_i] = VehicleRoute_i(ri);                                                                           \
                                    vehicle_routes_j[_vehicle_j].erase(vehicle_routes_j[_vehicle_j].begin() + route_j);                                                   \
                                }                                                                                                                                         \
                                else                                                                                                                                      \
                                {                                                                                                                                         \
                                    vehicle_routes_i[_vehicle_i][route_i] = VehicleRoute_i(ri);                                                                           \
                                    vehicle_routes_j[_vehicle_j][route_j] = VehicleRoute_j(rj);                                                                           \
                                }                                                                                                                                         \
                                                                                                                                                                          \
                                auto new_solution = this->construct(parent, truck_routes, drone_routes);                                                                  \
                                if (aspiration_criteria(new_solution) && (result == nullptr || new_solution->cost() < result->cost()))                                    \
                                {                                                                                                                                         \
                                    result = new_solution;                                                                                                                \
                                }                                                                                                                                         \
                                                                                                                                                                          \
                                /* Restore */                                                                                                                             \
                                if (ri_empty)                                                                                                                             \
                                {                                                                                                                                         \
                                    vehicle_routes_i[_vehicle_i].insert(vehicle_routes_i[_vehicle_i].begin() + route_i, solution->vehicle_routes_i[_vehicle_i][route_i]); \
                                    vehicle_routes_j[_vehicle_j][route_j] = solution->vehicle_routes_j[_vehicle_j][route_j];                                              \
                                }                                                                                                                                         \
                                else if (rj_empty)                                                                                                                        \
                                {                                                                                                                                         \
                                    vehicle_routes_j[_vehicle_j].insert(vehicle_routes_j[_vehicle_j].begin() + route_j, solution->vehicle_routes_j[_vehicle_j][route_j]); \
                                    vehicle_routes_i[_vehicle_i][route_i] = solution->vehicle_routes_i[_vehicle_i][route_i];                                              \
                                }                                                                                                                                         \
                                else                                                                                                                                      \
                                {                                                                                                                                         \
                                    vehicle_routes_i[_vehicle_i][route_i] = solution->vehicle_routes_i[_vehicle_i][route_i];                                              \
                                    vehicle_routes_j[_vehicle_j][route_j] = solution->vehicle_routes_j[_vehicle_j][route_j];                                              \
                                }                                                                                                                                         \
                            }                                                                                                                                             \
                        }                                                                                                                                                 \
                    }                                                                                                                                                     \
                }                                                                                                                                                         \
            }                                                                                                                                                             \
        }                                                                                                                                                                 \
    }

                    if (vehicle_i < problem->trucks_count)
                    {
                        if (vehicle_j < problem->trucks_count)
                        {
                            MODIFY_ROUTES(truck_routes, truck_routes);
                        }
                        else
                        {
                            MODIFY_ROUTES(truck_routes, drone_routes);
                        }
                    }
                    else
                    {
                        MODIFY_ROUTES(drone_routes, drone_routes);
                    }

#undef MODIFY_ROUTES
                }
            }

            return std::make_pair(result, std::make_pair(0, 0));
        }
    };
}
