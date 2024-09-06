#pragma once

#include "abc.hpp"

namespace d2d
{
    template <typename ST>
    class EjectionChain : public Neighborhood<ST, false>
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

            // std::cerr << "\e[31mInitial state:" << std::endl;
            // std::cerr << original_vehicle_routes_i << std::endl;
            // std::cerr << original_vehicle_routes_j << std::endl;
            // std::cerr << original_vehicle_routes_k << "\e[0m\n";

            for (std::size_t route_i = 0; route_i < original_vehicle_routes_i[_vehicle_i].size(); route_i++)
            {
                const auto &customers_i = original_vehicle_routes_i[_vehicle_i][route_i].customers();
                for (std::size_t i = 1; i + 1 < customers_i.size(); i++)
                {
                    if constexpr (std::is_same_v<_RT_I, TruckRoute> && std::is_same_v<_RT_J, DroneRoute>)
                    {
                        if (!problem->customers[customers_i[i]].dronable)
                        {
                            continue;
                        }
                    }

                    for (std::size_t route_j = 0; route_j < original_vehicle_routes_j[_vehicle_j].size(); route_j++)
                    {
                        if constexpr (std::is_same_v<_RT_I, _RT_J>)
                        {
                            if (_vehicle_i == _vehicle_j && route_i == route_j) /* same route */
                            {
                                continue;
                            }
                        }

                        const auto &customers_j = original_vehicle_routes_j[_vehicle_j][route_j].customers();
                        for (std::size_t jx = 1; jx < customers_j.size(); jx++)
                        {
                            for (std::size_t jy = 1; jy < customers_j.size(); jy++) /* Inserting customers_i[i] to customers_j[jx] increases size by 1 */
                            {
                                if (jx == jy) /* Route j is intact */
                                {
                                    continue;
                                }

                                if constexpr (std::is_same_v<_RT_J, TruckRoute> && std::is_same_v<_RT_K, DroneRoute>)
                                {
                                    if (!problem->customers[customers_j[jy - (jy > jx)]].dronable)
                                    {
                                        continue;
                                    }
                                }

                                /* Construct temporaries and temporary state lv1 */
                                std::vector<std::size_t> ri(customers_i), rj(customers_j);
                                ri.erase(ri.begin() + i);
                                rj.insert(rj.begin() + jx, customers_i[i]);
                                rj.erase(rj.begin() + jy);

                                std::size_t insert_k = customers_j[jy - (jy > jx)];
                                vehicle_routes_j[_vehicle_j][route_j] = _RT_J(rj);
                                if (ri.size() == 2)
                                {
                                    vehicle_routes_i[_vehicle_i].erase(vehicle_routes_i[_vehicle_i].begin() + route_i);
                                }
                                else
                                {
                                    vehicle_routes_i[_vehicle_i][route_i] = _RT_I(ri);
                                }

                                if constexpr (std::is_same_v<_RT_K, DroneRoute>)
                                {
                                    /* Construct a new route of vehicle_k (temporary state lv2) */
                                    vehicle_routes_k[_vehicle_k].emplace_back(std::vector<std::size_t>{0, insert_k, 0});

                                    auto new_solution = this->construct(parent, truck_routes, drone_routes);
                                    if (aspiration_criteria(new_solution) && (result == nullptr || new_solution->cost() < result->cost()))
                                    {
                                        result = new_solution;
                                    }

                                    /* Restore temporary state lv1 */
                                    vehicle_routes_k[_vehicle_k].pop_back();
                                }

                                // std::cerr << "1. Restore temporary state " << vehicle_routes_i << " " << vehicle_routes_j << std::endl;
                                /* Swap customers between 3 existing routes */
                                bool same_ik = (vehicle_i == vehicle_k);
                                for (std::size_t route_k = 0; route_k < original_vehicle_routes_k[_vehicle_k].size() - same_ik; route_k++)
                                {
                                    if constexpr (std::is_same_v<_RT_J, _RT_K>)
                                    {
                                        if (_vehicle_j == _vehicle_k && route_j == route_k) /* same route */
                                        {
                                            continue;
                                        }
                                    }

                                    if constexpr (std::is_same_v<_RT_K, _RT_I>)
                                    {
                                        if (_vehicle_k == _vehicle_i && route_k == route_i) /* same route */
                                        {
                                            continue;
                                        }
                                    }

                                    if constexpr (std::is_same_v<_RT_K, TruckRoute>)
                                    {
                                        std::size_t route_k_new = route_k - (ri.size() == 2 && same_ik && route_k >= route_i);
                                        const auto &customers_k = original_vehicle_routes_k[_vehicle_k][route_k].customers();
                                        for (std::size_t k = 1; k < customers_k.size(); k++)
                                        {
                                            /* Insert to position k */
                                            std::vector<std::size_t> rk(customers_k);
                                            rk.insert(rk.begin() + k, insert_k);

                                            /* Temporary modify (temporary state lv2) */
                                            vehicle_routes_k[_vehicle_k][route_k_new] = _RT_K(rk);

                                            auto new_solution = this->construct(parent, truck_routes, drone_routes);
                                            if (aspiration_criteria(new_solution) && (result == nullptr || new_solution->cost() < result->cost()))
                                            {
                                                result = new_solution;
                                            }

                                            /* Restore temporary state lv1 */
                                            vehicle_routes_k[_vehicle_k][route_k_new] = original_vehicle_routes_k[_vehicle_k][route_k];
                                        }
                                    }
                                }

                                /* Restore */
                                vehicle_routes_i[_vehicle_i] = original_vehicle_routes_i[_vehicle_i];
                                if (vehicle_i != vehicle_j)
                                {
                                    vehicle_routes_j[_vehicle_j][route_j] = original_vehicle_routes_j[_vehicle_j][route_j];
                                }
                                // std::cerr << "3. Restore original state " << vehicle_routes_i << " " << vehicle_routes_j << std::endl;
                            }
                        }
                    }
                }
            }
        }

    public:
        std::string label() const override
        {
            return "Ejection chain";
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
                for (std::size_t vehicle_j = 0; vehicle_j < problem->trucks_count + problem->drones_count; vehicle_j++)
                {
                    for (std::size_t vehicle_k = 0; vehicle_k < problem->trucks_count + problem->drones_count; vehicle_k++)
                    {
                        if (vehicle_i < problem->trucks_count)
                        {
                            if (vehicle_j < problem->trucks_count)
                            {
                                if (vehicle_k < problem->trucks_count)
                                {
                                    _inter_route_internal<TruckRoute, TruckRoute, TruckRoute>(
                                        solution,
                                        aspiration_criteria,
                                        parent,
                                        result,
                                        truck_routes,
                                        drone_routes,
                                        vehicle_i,
                                        vehicle_j,
                                        vehicle_k);
                                }
                                else
                                {
                                    _inter_route_internal<TruckRoute, TruckRoute, DroneRoute>(
                                        solution,
                                        aspiration_criteria,
                                        parent,
                                        result,
                                        truck_routes,
                                        drone_routes,
                                        vehicle_i,
                                        vehicle_j,
                                        vehicle_k);
                                }
                            }
                            else
                            {
                                if (vehicle_k < problem->trucks_count)
                                {
                                    _inter_route_internal<TruckRoute, DroneRoute, TruckRoute>(
                                        solution,
                                        aspiration_criteria,
                                        parent,
                                        result,
                                        truck_routes,
                                        drone_routes,
                                        vehicle_i,
                                        vehicle_j,
                                        vehicle_k);
                                }
                                else
                                {
                                    _inter_route_internal<TruckRoute, DroneRoute, DroneRoute>(
                                        solution,
                                        aspiration_criteria,
                                        parent,
                                        result,
                                        truck_routes,
                                        drone_routes,
                                        vehicle_i,
                                        vehicle_j,
                                        vehicle_k);
                                }
                            }
                        }
                        else
                        {
                            if (vehicle_j < problem->trucks_count)
                            {
                                if (vehicle_k < problem->trucks_count)
                                {
                                    _inter_route_internal<DroneRoute, TruckRoute, TruckRoute>(
                                        solution,
                                        aspiration_criteria,
                                        parent,
                                        result,
                                        truck_routes,
                                        drone_routes,
                                        vehicle_i,
                                        vehicle_j,
                                        vehicle_k);
                                }
                                else
                                {
                                    _inter_route_internal<DroneRoute, TruckRoute, DroneRoute>(
                                        solution,
                                        aspiration_criteria,
                                        parent,
                                        result,
                                        truck_routes,
                                        drone_routes,
                                        vehicle_i,
                                        vehicle_j,
                                        vehicle_k);
                                }
                            }
                            else
                            {
                                if (vehicle_k < problem->trucks_count)
                                {
                                    _inter_route_internal<DroneRoute, DroneRoute, TruckRoute>(
                                        solution,
                                        aspiration_criteria,
                                        parent,
                                        result,
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
                                        parent,
                                        result,
                                        truck_routes,
                                        drone_routes,
                                        vehicle_i,
                                        vehicle_j,
                                        vehicle_k);
                                }
                            }
                        }
                    }
                }
            }

            return std::make_pair(result, std::vector<std::size_t>());
        }
    };
}
