#pragma once

#include "abc.hpp"

namespace d2d
{
    template <typename ST>
    class CrossExchange : public Neighborhood<ST, false>
    {
    private:
        template <typename _RT_I, typename _RT_J, std::enable_if_t<is_route_v<_RT_I, _RT_J>, bool> = true>
        void _inter_route_internal(
            const std::shared_ptr<ST> solution,
            const std::function<bool(const std::shared_ptr<ST>)> &aspiration_criteria,
            const std::shared_ptr<ParentInfo<ST>> parent,
            std::shared_ptr<ST> &result,
            std::vector<std::vector<TruckRoute>> &truck_routes,
            std::vector<std::vector<DroneRoute>> &drone_routes,
            const std::size_t &vehicle_i,
            const std::size_t &vehicle_j)
        {
            auto problem = Problem::get_instance();

            std::size_t _vehicle_i = utils::ternary<std::is_same_v<_RT_I, TruckRoute>>(vehicle_i, vehicle_i - problem->trucks_count);
            std::size_t _vehicle_j = utils::ternary<std::is_same_v<_RT_J, TruckRoute>>(vehicle_j, vehicle_j - problem->trucks_count);

            auto &vehicle_routes_i = utils::match_type<std::vector<std::vector<_RT_I>>>(truck_routes, drone_routes);
            auto &vehicle_routes_j = utils::match_type<std::vector<std::vector<_RT_J>>>(truck_routes, drone_routes);
            auto &original_vehicle_routes_i = utils::match_type<std::vector<std::vector<_RT_I>>>(solution->truck_routes, solution->drone_routes);
            auto &original_vehicle_routes_j = utils::match_type<std::vector<std::vector<_RT_J>>>(solution->truck_routes, solution->drone_routes);

            for (std::size_t route_i = 0; route_i < original_vehicle_routes_i[_vehicle_i].size(); route_i++)
            {
                for (std::size_t route_j = 0; route_j < original_vehicle_routes_j[_vehicle_j].size(); route_j++)
                {
                    if constexpr (std::is_same_v<_RT_I, _RT_J>)
                    {
                        if (_vehicle_i == _vehicle_j && route_i == route_j) /* same route */
                        {
                            continue;
                        }
                    }

                    const auto &customers_i = original_vehicle_routes_i[_vehicle_i][route_i].customers();
                    const auto &customers_j = original_vehicle_routes_j[_vehicle_j][route_j].customers();

                    for (std::size_t i = 1; i + 1 < customers_i.size(); i++)
                    {
                        for (std::size_t j = 1; j + 1 < customers_j.size(); j++)
                        {
                            for (std::size_t ix = i; ix < customers_i.size(); ix++)
                            {
                                for (std::size_t jx = j; jx < customers_j.size(); jx++)
                                {
                                    if constexpr (std::is_same_v<_RT_I, DroneRoute> && std::is_same_v<_RT_J, TruckRoute>)
                                    {
                                        if (std::any_of(
                                                customers_j.begin() + j, customers_j.begin() + jx,
                                                [&problem](const std::size_t &c)
                                                { return !problem->customers[c].dronable; }))
                                        {
                                            continue;
                                        }
                                    }

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

                                    /* Swap [i, ix) of route_i and [j, jx) of route_j */
                                    std::vector<std::size_t> ri(customers_i.begin(), customers_i.begin() + i);
                                    std::vector<std::size_t> rj(customers_j.begin(), customers_j.begin() + j);

                                    ri.insert(ri.end(), customers_j.begin() + j, customers_j.begin() + jx);
                                    rj.insert(rj.end(), customers_i.begin() + i, customers_i.begin() + ix);

                                    ri.insert(ri.end(), customers_i.begin() + ix, customers_i.end());
                                    rj.insert(rj.end(), customers_j.begin() + jx, customers_j.end());

                                    bool ri_empty = (ri.size() == 2), rj_empty = (rj.size() == 2); /* Note: These values can't be true at the same time */
                                    if (ri_empty)
                                    {
                                        vehicle_routes_j[_vehicle_j][route_j] = _RT_J(rj);
                                        vehicle_routes_i[_vehicle_i].erase(vehicle_routes_i[_vehicle_i].begin() + route_i);
                                    }
                                    else if (rj_empty)
                                    {
                                        vehicle_routes_i[_vehicle_i][route_i] = _RT_I(ri);
                                        vehicle_routes_j[_vehicle_j].erase(vehicle_routes_j[_vehicle_j].begin() + route_j);
                                    }
                                    else
                                    {
                                        vehicle_routes_i[_vehicle_i][route_i] = _RT_I(ri);
                                        vehicle_routes_j[_vehicle_j][route_j] = _RT_J(rj);
                                    }

                                    auto new_solution = this->construct(parent, truck_routes, drone_routes);
                                    if (aspiration_criteria(new_solution) && (result == nullptr || new_solution->cost() < result->cost()))
                                    {
                                        result = new_solution;
                                    }

                                    /* Restore */
                                    if (ri_empty)
                                    {
                                        vehicle_routes_i[_vehicle_i].insert(vehicle_routes_i[_vehicle_i].begin() + route_i, original_vehicle_routes_i[_vehicle_i][route_i]);
                                        vehicle_routes_j[_vehicle_j][route_j] = original_vehicle_routes_j[_vehicle_j][route_j];
                                    }
                                    else if (rj_empty)
                                    {
                                        vehicle_routes_j[_vehicle_j].insert(vehicle_routes_j[_vehicle_j].begin() + route_j, original_vehicle_routes_j[_vehicle_j][route_j]);
                                        vehicle_routes_i[_vehicle_i][route_i] = original_vehicle_routes_i[_vehicle_i][route_i];
                                    }
                                    else
                                    {
                                        vehicle_routes_i[_vehicle_i][route_i] = original_vehicle_routes_i[_vehicle_i][route_i];
                                        vehicle_routes_j[_vehicle_j][route_j] = original_vehicle_routes_j[_vehicle_j][route_j];
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
            return "CROSS-exchange";
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
                    if (vehicle_i < problem->trucks_count)
                    {
                        if (vehicle_j < problem->trucks_count)
                        {
                            _inter_route_internal<TruckRoute, TruckRoute>(
                                solution,
                                aspiration_criteria,
                                parent, result,
                                truck_routes,
                                drone_routes,
                                vehicle_i,
                                vehicle_j);
                        }
                        else
                        {
                            _inter_route_internal<TruckRoute, DroneRoute>(
                                solution,
                                aspiration_criteria,
                                parent, result,
                                truck_routes,
                                drone_routes,
                                vehicle_i,
                                vehicle_j);
                        }
                    }
                    else
                    {
                        _inter_route_internal<DroneRoute, DroneRoute>(
                            solution,
                            aspiration_criteria,
                            parent, result,
                            truck_routes,
                            drone_routes,
                            vehicle_i,
                            vehicle_j);
                    }
                }
            }

            return std::make_pair(result, std::vector<std::size_t>());
        }
    };
}
