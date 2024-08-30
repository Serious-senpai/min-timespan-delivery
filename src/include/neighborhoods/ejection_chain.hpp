#pragma once

#include "abc.hpp"

namespace d2d
{
    template <typename ST>
    class EjectionChain : public Neighborhood<ST, false>
    {
    public:
        std::string label() const override
        {
            return "Ejection chain";
        }

        std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> intra_route(
            const std::shared_ptr<ST> solution,
            const std::function<bool(const std::shared_ptr<ST>)> &aspiration_criteria) override
        {
            return std::make_pair(nullptr, std::make_pair(0, 0));
        }

        std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> inter_route(
            const std::shared_ptr<ST> solution,
            const std::function<bool(const std::shared_ptr<ST>)> &aspiration_criteria) override
        {
            auto problem = Problem::get_instance();
            auto parent = this->parent_ptr(solution);
            std::shared_ptr<ST> result;

            std::vector<std::vector<TruckRoute>> truck_routes(solution->truck_routes);
            std::vector<std::vector<DroneRoute>> drone_routes(solution->drone_routes);

            for (std::size_t vehicle_i = 0; vehicle_i < problem->trucks_count + problem->drones_count; vehicle_i++)
            {
                for (std::size_t vehicle_j = 0; vehicle_j < problem->trucks_count + problem->drones_count; vehicle_j++)
                {
                    for (std::size_t vehicle_k = 0; vehicle_k < problem->trucks_count + problem->drones_count; vehicle_k++)
                    {
#define MODIFY_ROUTES(vehicle_routes_i, vehicle_routes_j, vehicle_routes_k)                                                                               \
    {                                                                                                                                                     \
        std::size_t _vehicle_i = vehicle_i, _vehicle_j = vehicle_j, _vehicle_k = vehicle_k;                                                               \
        if constexpr (std::is_same_v<std::remove_cvref_t<decltype(solution->vehicle_routes_i)>, std::vector<std::vector<DroneRoute>>>)                    \
        {                                                                                                                                                 \
            _vehicle_i -= problem->trucks_count;                                                                                                          \
        }                                                                                                                                                 \
                                                                                                                                                          \
        if constexpr (std::is_same_v<std::remove_cvref_t<decltype(solution->vehicle_routes_j)>, std::vector<std::vector<DroneRoute>>>)                    \
        {                                                                                                                                                 \
            _vehicle_j -= problem->trucks_count;                                                                                                          \
        }                                                                                                                                                 \
                                                                                                                                                          \
        if constexpr (std::is_same_v<std::remove_cvref_t<decltype(solution->vehicle_routes_k)>, std::vector<std::vector<DroneRoute>>>)                    \
        {                                                                                                                                                 \
            _vehicle_k -= problem->trucks_count;                                                                                                          \
        }                                                                                                                                                 \
                                                                                                                                                          \
        for (std::size_t route_i = 0; route_i < solution->vehicle_routes_i[_vehicle_i].size(); route_i++)                                                 \
        {                                                                                                                                                 \
            for (std::size_t route_j = 0; route_j < solution->vehicle_routes_j[_vehicle_j].size(); route_j++)                                             \
            {                                                                                                                                             \
                for (std::size_t route_k = 0; route_k < solution->vehicle_routes_k[_vehicle_k].size(); route_k++)                                         \
                {                                                                                                                                         \
                    using VehicleRoute_i = std::remove_cvref_t<decltype(vehicle_routes_i[_vehicle_i][route_i])>;                                          \
                    using VehicleRoute_j = std::remove_cvref_t<decltype(vehicle_routes_j[_vehicle_j][route_j])>;                                          \
                    using VehicleRoute_k = std::remove_cvref_t<decltype(vehicle_routes_k[_vehicle_k][route_k])>;                                          \
                                                                                                                                                          \
                    if constexpr (std::is_same_v<VehicleRoute_i, VehicleRoute_j>)                                                                         \
                    {                                                                                                                                     \
                        if (_vehicle_i == _vehicle_j && route_i == route_j) /* same route */                                                              \
                        {                                                                                                                                 \
                            continue;                                                                                                                     \
                        }                                                                                                                                 \
                    }                                                                                                                                     \
                                                                                                                                                          \
                    if constexpr (std::is_same_v<VehicleRoute_j, VehicleRoute_k>)                                                                         \
                    {                                                                                                                                     \
                        if (_vehicle_j == _vehicle_k && route_j == route_k) /* same route */                                                              \
                        {                                                                                                                                 \
                            continue;                                                                                                                     \
                        }                                                                                                                                 \
                    }                                                                                                                                     \
                                                                                                                                                          \
                    if constexpr (std::is_same_v<VehicleRoute_k, VehicleRoute_i>)                                                                         \
                    {                                                                                                                                     \
                        if (_vehicle_k == _vehicle_i && route_k == route_i) /* same route */                                                              \
                        {                                                                                                                                 \
                            continue;                                                                                                                     \
                        }                                                                                                                                 \
                    }                                                                                                                                     \
                                                                                                                                                          \
                    const std::vector<std::size_t> &customers_i = solution->vehicle_routes_i[_vehicle_i][route_i].customers();                            \
                    const std::vector<std::size_t> &customers_j = solution->vehicle_routes_j[_vehicle_j][route_j].customers();                            \
                    const std::vector<std::size_t> &customers_k = solution->vehicle_routes_k[_vehicle_k][route_k].customers();                            \
                                                                                                                                                          \
                    for (std::size_t i = 1; i + 1 < customers_i.size(); i++)                                                                              \
                    {                                                                                                                                     \
                        for (std::size_t jx = 1; jx < customers_j.size(); jx++)                                                                           \
                        {                                                                                                                                 \
                            for (std::size_t jy = 1; jy < customers_j.size(); jy++) /* Inserting customers_i[i] to customers_j[jx] increases size by 1 */ \
                            {                                                                                                                             \
                                if (jx == jy) /* Route j is intact */                                                                                     \
                                {                                                                                                                         \
                                    continue;                                                                                                             \
                                }                                                                                                                         \
                                                                                                                                                          \
                                for (std::size_t k = 1; k < customers_k.size(); k++)                                                                      \
                                {                                                                                                                         \
                                    if constexpr (std::is_same_v<VehicleRoute_i, TruckRoute> && std::is_same_v<VehicleRoute_j, DroneRoute>)               \
                                    {                                                                                                                     \
                                        if (!problem->customers[customers_i[i]].dronable)                                                                 \
                                        {                                                                                                                 \
                                            continue;                                                                                                     \
                                        }                                                                                                                 \
                                    }                                                                                                                     \
                                                                                                                                                          \
                                    if constexpr (std::is_same_v<VehicleRoute_j, TruckRoute> && std::is_same_v<VehicleRoute_k, DroneRoute>)               \
                                    {                                                                                                                     \
                                        if (!problem->customers[customers_j[jy - (jy > jx)]].dronable)                                                    \
                                        {                                                                                                                 \
                                            continue;                                                                                                     \
                                        }                                                                                                                 \
                                    }                                                                                                                     \
                                                                                                                                                          \
                                    std::vector<std::size_t> ri(customers_i), rj(customers_j), rk(customers_k);                                           \
                                    ri.erase(ri.begin() + i);                                                                                             \
                                    rj.insert(rj.begin() + jx, customers_i[i]);                                                                           \
                                    rj.erase(rj.begin() + jy);                                                                                            \
                                    rk.insert(rk.begin() + k, customers_j[jy - (jy > jx)]);                                                               \
                                                                                                                                                          \
                                    /* Perform assignment first before doing any deletion */                                                              \
                                    vehicle_routes_j[_vehicle_j][route_j] = VehicleRoute_j(rj);                                                           \
                                    vehicle_routes_k[_vehicle_k][route_k] = VehicleRoute_k(rk);                                                           \
                                    if (ri.size() == 2)                                                                                                   \
                                    {                                                                                                                     \
                                        vehicle_routes_i[_vehicle_i].erase(vehicle_routes_i[_vehicle_i].begin() + route_i);                               \
                                    }                                                                                                                     \
                                    else                                                                                                                  \
                                    {                                                                                                                     \
                                        vehicle_routes_i[_vehicle_i][route_i] = VehicleRoute_i(ri);                                                       \
                                    }                                                                                                                     \
                                                                                                                                                          \
                                    auto new_solution = this->construct(parent, truck_routes, drone_routes);                                              \
                                    if (aspiration_criteria(new_solution) && (result == nullptr || new_solution->cost() < result->cost()))                \
                                    {                                                                                                                     \
                                        result = new_solution;                                                                                            \
                                    }                                                                                                                     \
                                                                                                                                                          \
                                    /* Restore */                                                                                                         \
                                    vehicle_routes_i[_vehicle_i] = solution->vehicle_routes_i[_vehicle_i];                                                \
                                    if (vehicle_i != vehicle_j)                                                                                           \
                                    {                                                                                                                     \
                                        vehicle_routes_j[_vehicle_j][route_j] = solution->vehicle_routes_j[_vehicle_j][route_j];                          \
                                    }                                                                                                                     \
                                    if (vehicle_i != vehicle_k)                                                                                           \
                                    {                                                                                                                     \
                                        vehicle_routes_k[_vehicle_k][route_k] = solution->vehicle_routes_k[_vehicle_k][route_k];                          \
                                    }                                                                                                                     \
                                }                                                                                                                         \
                            }                                                                                                                             \
                        }                                                                                                                                 \
                    }                                                                                                                                     \
                }                                                                                                                                         \
            }                                                                                                                                             \
        }                                                                                                                                                 \
    }

                        if (vehicle_i < problem->trucks_count)
                        {
                            if (vehicle_j < problem->trucks_count)
                            {
                                if (vehicle_k < problem->trucks_count)
                                {
                                    MODIFY_ROUTES(truck_routes, truck_routes, truck_routes);
                                }
                                else
                                {
                                    MODIFY_ROUTES(truck_routes, truck_routes, drone_routes);
                                }
                            }
                            else
                            {
                                if (vehicle_k < problem->trucks_count)
                                {
                                    MODIFY_ROUTES(truck_routes, drone_routes, truck_routes);
                                }
                                else
                                {
                                    MODIFY_ROUTES(truck_routes, drone_routes, drone_routes);
                                }
                            }
                        }
                        else
                        {
                            if (vehicle_j < problem->trucks_count)
                            {
                                if (vehicle_k < problem->trucks_count)
                                {
                                    MODIFY_ROUTES(drone_routes, truck_routes, truck_routes);
                                }
                                else
                                {
                                    MODIFY_ROUTES(drone_routes, truck_routes, drone_routes);
                                }
                            }
                            else
                            {
                                if (vehicle_k < problem->trucks_count)
                                {
                                    MODIFY_ROUTES(drone_routes, drone_routes, truck_routes);
                                }
                                else
                                {
                                    MODIFY_ROUTES(drone_routes, drone_routes, drone_routes);
                                }
                            }
                        }

#undef MODIFY_ROUTES
                    }
                }
            }

            return std::make_pair(result, std::make_pair(0, 0));
        }
    };
}
