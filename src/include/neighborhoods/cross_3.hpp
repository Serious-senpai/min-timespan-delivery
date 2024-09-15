#pragma once

#include "abc.hpp"

namespace d2d
{
    template <typename ST>
    class CrossExchange_3 : public Neighborhood<ST, false>
    {
    private:
        template <typename _RT_I, typename _RT_J, typename _RT_K, std::enable_if_t<is_route_v<_RT_I, _RT_J, _RT_K>, bool> = true>
        void _inter_route_internal(
            const std::shared_ptr<ST> solution,
            const std::function<bool(const std::shared_ptr<ST>)> &aspiration_criteria,
            const std::shared_ptr<ParentInfo<ST>> parent,
            std::shared_ptr<ST> &result,
            std::vector<std::vector<TruckRoute>> &truck_routes,
            std::vector<std::vector<DroneRoute>> &drone_routes,
            const std::size_t &vehicle_i,
            const std::size_t &vehicle_j,
            const std::size_t &vehicle_k)
        {
            auto problem = Problem::get_instance();

            std::size_t _vehicle_i = utils::ternary<std::is_same_v<_RT_I, TruckRoute>>(vehicle_i, vehicle_i - problem->trucks_count);
            std::size_t _vehicle_j = utils::ternary<std::is_same_v<_RT_J, TruckRoute>>(vehicle_j, vehicle_j - problem->trucks_count);
            std::size_t _vehicle_k = utils::ternary<std::is_same_v<_RT_K, TruckRoute>>(vehicle_k, vehicle_k - problem->trucks_count);

            auto &vehicle_routes_i = utils::match_type<std::vector<std::vector<_RT_I>>>(truck_routes, drone_routes);
            auto &vehicle_routes_j = utils::match_type<std::vector<std::vector<_RT_J>>>(truck_routes, drone_routes);
            auto &vehicle_routes_k = utils::match_type<std::vector<std::vector<_RT_K>>>(truck_routes, drone_routes);

            auto &original_vehicle_routes_i = utils::match_type<std::vector<std::vector<_RT_I>>>(solution->truck_routes, solution->drone_routes);
            auto &original_vehicle_routes_j = utils::match_type<std::vector<std::vector<_RT_J>>>(solution->truck_routes, solution->drone_routes);
            auto &original_vehicle_routes_k = utils::match_type<std::vector<std::vector<_RT_K>>>(solution->truck_routes, solution->drone_routes);

            for (std::size_t route_i = 0; route_i < original_vehicle_routes_i[_vehicle_i].size(); route_i++)
            {
                for (std::size_t route_j = 0; route_j < original_vehicle_routes_j[_vehicle_j].size(); route_j++)
                {
                    if constexpr (std::is_same_v<_RT_I, _RT_J>)
                    {
                        if (_vehicle_i == _vehicle_j && route_i >= route_j)
                        {
                            route_j = route_i;
                            continue;
                        }
                    }

                    for (std::size_t route_k = 0; route_k < original_vehicle_routes_k[_vehicle_k].size(); route_k++)
                    {
                        if constexpr (std::is_same_v<_RT_J, _RT_K>)
                        {
                            if (_vehicle_j == _vehicle_k && route_j >= route_k)
                            {
                                route_k = route_j;
                                continue;
                            }
                        }

                        const auto &customers_i = original_vehicle_routes_i[_vehicle_i][route_i].customers();
                        const auto &customers_j = original_vehicle_routes_j[_vehicle_j][route_j].customers();
                        const auto &customers_k = original_vehicle_routes_k[_vehicle_k][route_k].customers();

                        for (std::size_t i = 1; i < customers_i.size(); i++)
                        {
                            for (std::size_t ix = i; ix < customers_i.size(); ix++)
                            {
                                if constexpr (std::is_same_v<_RT_I, TruckRoute> && std::is_same_v<_RT_J, DroneRoute>)
                                {
                                    if (std::any_of(
                                            customers_i.begin() + i, customers_i.begin() + ix,
                                            [&problem](const std::size_t &c)
                                            { return !problem->customers[c].dronable; }))
                                    {
                                        continue;
                                    }
                                }

                                for (std::size_t j = 1; j < customers_j.size(); j++)
                                {
                                    for (std::size_t jx = j; jx < customers_j.size(); jx++)
                                    {
                                        if constexpr (std::is_same_v<_RT_J, TruckRoute> && std::is_same_v<_RT_K, DroneRoute>)
                                        {
                                            if (std::any_of(
                                                    customers_j.begin() + j, customers_j.begin() + jx,
                                                    [&problem](const std::size_t &c)
                                                    { return !problem->customers[c].dronable; }))
                                            {
                                                continue;
                                            }
                                        }

                                        for (std::size_t k = 1; k < customers_k.size(); k++)
                                        {
                                            for (std::size_t kx = k; kx < customers_k.size(); kx++)
                                            {
                                                /* Swap [i, ix) of route_i, [j, jx) of route_j and [k, kx) of route_k (forward) */
                                                std::vector<std::size_t> ri(customers_i.begin(), customers_i.begin() + i);
                                                std::vector<std::size_t> rj(customers_j.begin(), customers_j.begin() + j);
                                                std::vector<std::size_t> rk(customers_k.begin(), customers_k.begin() + k);

                                                ri.insert(ri.end(), customers_k.begin() + k, customers_k.begin() + kx);
                                                rj.insert(rj.end(), customers_i.begin() + i, customers_i.begin() + ix);
                                                rk.insert(rk.end(), customers_j.begin() + j, customers_j.begin() + jx);

                                                ri.insert(ri.end(), customers_i.begin() + ix, customers_i.end());
                                                rj.insert(rj.end(), customers_j.begin() + jx, customers_j.end());
                                                rk.insert(rk.end(), customers_k.begin() + kx, customers_k.end());

                                                if constexpr (std::is_same_v<_RT_I, DroneRoute>)
                                                {
                                                    if (ri.size() > 3)
                                                    {
                                                        continue;
                                                    }
                                                }
                                                if constexpr (std::is_same_v<_RT_J, DroneRoute>)
                                                {
                                                    if (rj.size() > 3)
                                                    {
                                                        continue;
                                                    }
                                                }
                                                if constexpr (std::is_same_v<_RT_K, DroneRoute>)
                                                {
                                                    if (rk.size() > 3)
                                                    {
                                                        continue;
                                                    }
                                                }

                                                // Keep in mind that index of route i < j < k
                                                if (rk.size() == 2)
                                                {
                                                    vehicle_routes_k[_vehicle_k].erase(vehicle_routes_k[_vehicle_k].begin() + route_k);
                                                }
                                                else
                                                {
                                                    vehicle_routes_k[_vehicle_k][route_k] = _RT_K(rk);
                                                }
                                                if (rj.size() == 2)
                                                {
                                                    vehicle_routes_j[_vehicle_j].erase(vehicle_routes_j[_vehicle_j].begin() + route_j);
                                                }
                                                else
                                                {
                                                    vehicle_routes_j[_vehicle_j][route_j] = _RT_J(rj);
                                                }
                                                if (ri.size() == 2)
                                                {
                                                    vehicle_routes_i[_vehicle_i].erase(vehicle_routes_i[_vehicle_i].begin() + route_i);
                                                }
                                                else
                                                {
                                                    vehicle_routes_i[_vehicle_i][route_i] = _RT_I(ri);
                                                }

                                                auto new_solution = this->construct(parent, truck_routes, drone_routes);
                                                if (aspiration_criteria(new_solution) && (result == nullptr || new_solution->cost() < result->cost()))
                                                {
                                                    result = new_solution;
                                                }

                                                /* Restore */
                                                vehicle_routes_i[_vehicle_i] = original_vehicle_routes_i[_vehicle_i];
                                                if (vehicle_i != vehicle_j)
                                                {
                                                    vehicle_routes_j[_vehicle_j] = original_vehicle_routes_j[_vehicle_j];
                                                }
                                                if (vehicle_j != vehicle_k)
                                                {
                                                    vehicle_routes_k[_vehicle_k] = original_vehicle_routes_k[_vehicle_k];
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

    public:
        std::string label() const override
        {
            return "CROSS-exchange 3";
        }

        std::pair<std::shared_ptr<ST>, std::vector<std::size_t>> intra_route(
            const std::shared_ptr<ST> solution,
            const std::function<bool(const std::shared_ptr<ST>)> &aspiration_criteria) override
        {
            return std::make_pair(nullptr, std::vector<std::size_t>());
        }

        std::pair<std::shared_ptr<ST>, std::vector<std::size_t>> inter_route(
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
                for (std::size_t vehicle_j = vehicle_i; vehicle_j < problem->trucks_count + problem->drones_count; vehicle_j++)
                {
                    for (std::size_t vehicle_k = vehicle_j; vehicle_k < problem->trucks_count + problem->drones_count; vehicle_k++)
                    {
                        if (vehicle_k < problem->trucks_count)
                        {
                            _inter_route_internal<TruckRoute, TruckRoute, TruckRoute>(
                                solution,
                                aspiration_criteria,
                                parent, result,
                                truck_routes,
                                drone_routes,
                                vehicle_i,
                                vehicle_j,
                                vehicle_k);
                        }
                        else if (vehicle_j < problem->trucks_count)
                        {
                            _inter_route_internal<TruckRoute, TruckRoute, DroneRoute>(
                                solution,
                                aspiration_criteria,
                                parent, result,
                                truck_routes,
                                drone_routes,
                                vehicle_i,
                                vehicle_j,
                                vehicle_k);
                        }
                        else if (vehicle_i < problem->trucks_count)
                        {
                            _inter_route_internal<TruckRoute, DroneRoute, DroneRoute>(
                                solution,
                                aspiration_criteria,
                                parent, result,
                                truck_routes,
                                drone_routes,
                                vehicle_i,
                                vehicle_j,
                                vehicle_k);
                        }
                        else
                        {
                            _inter_route_internal<DroneRoute, DroneRoute, DroneRoute>(
                                solution,
                                aspiration_criteria,
                                parent, result,
                                truck_routes,
                                drone_routes,
                                vehicle_i,
                                vehicle_j,
                                vehicle_k);
                        }
                    }
                }
            }

            return std::make_pair(result, std::vector<std::size_t>());
        }
    };
}
