#pragma once

#include "abc.hpp"

namespace d2d
{
    template <typename ST, std::size_t X, std::size_t Y, std::enable_if_t<(X >= Y && X != 0), bool> = true>
    class _BaseMoveXY : public Neighborhood<ST, true>
    {
    private:
        template <typename _RT_I, typename _RT_J, std::enable_if_t<std::conjunction_v<is_route<_RT_I>, is_route<_RT_J>>, bool> = true>
        void _inter_route_internal(
            const std::shared_ptr<ST> solution,
            const std::function<bool(const std::shared_ptr<ST>)> &aspiration_criteria,
            const std::shared_ptr<ParentInfo<ST>> parent,
            std::shared_ptr<ST> &result,
            std::pair<std::size_t, std::size_t> &tabu_pair,
            std::vector<std::vector<TruckRoute>> &truck_routes,
            std::vector<std::vector<DroneRoute>> &drone_routes,
            const std::size_t &vehicle_i,
            const std::size_t &vehicle_j)
        {
            if constexpr (X != Y && (std::is_same_v<_RT_I, DroneRoute> || std::is_same_v<_RT_J, DroneRoute>))
            {
                return;
            }

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
                    const auto &customers_i = original_vehicle_routes_i[_vehicle_i][route_i].customers();
                    const auto &customers_j = original_vehicle_routes_j[_vehicle_j][route_j].customers();
                    for (std::size_t i = 1; i + X < customers_i.size(); i++)
                    {
                        for (std::size_t j = 1; j + Y < customers_j.size(); j++)
                        {
                            if constexpr (std::is_same_v<_RT_I, _RT_J>)
                            {
                                if (_vehicle_i == _vehicle_j && route_i == route_j) /* same route */
                                {
                                    continue;
                                }
                            }

                            /* Swap [i, i + X) of route i and [j, j + Y) of route j */

                            std::vector<std::size_t> ri(customers_i.begin(), customers_i.begin() + i);
                            std::vector<std::size_t> rj(customers_j.begin(), customers_j.begin() + j);

                            ri.insert(ri.end(), customers_j.begin() + j, customers_j.begin() + (j + Y));
                            rj.insert(rj.end(), customers_i.begin() + i, customers_i.begin() + (i + X));

                            ri.insert(ri.end(), customers_i.begin() + (i + X), customers_i.end());
                            rj.insert(rj.end(), customers_j.begin() + (j + Y), customers_j.end());

                            if constexpr (std::is_same_v<_RT_I, DroneRoute> && std::is_same_v<_RT_J, TruckRoute>)
                            {
                                if (std::any_of(
                                        customers_j.begin() + j, customers_j.begin() + (j + Y),
                                        [&problem](const std::size_t &c)
                                        { return !problem->customers[c].dronable; }))
                                {
                                    continue;
                                }
                            }

                            if constexpr (std::is_same_v<_RT_I, TruckRoute> && std::is_same_v<_RT_J, DroneRoute>)
                            {
                                if (std::any_of(
                                        customers_i.begin() + i, customers_i.begin() + (i + X),
                                        [&problem](const std::size_t &c)
                                        { return !problem->customers[c].dronable; }))
                                {
                                    continue;
                                }
                            }

                            /* Temporary modify */
                            bool ri_empty = (ri.size() == 2), rj_empty = (rj.size() == 2); /* Note: At least 1 flag is false */
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
                            if ((aspiration_criteria(new_solution) || !this->is_tabu(customers_i[i], customers_j[j])) &&
                                (result == nullptr || new_solution->cost() < result->cost()))
                            {
                                result = new_solution;
                                tabu_pair = std::make_pair(customers_i[i], customers_j[j]);
                            }

                            /* Restore */
                            vehicle_routes_i[_vehicle_i] = original_vehicle_routes_i[_vehicle_i];
                            vehicle_routes_j[_vehicle_j] = original_vehicle_routes_j[_vehicle_j];
                        }
                    }
                }
            }
        }

        template <typename _RT_Src, std::enable_if_t<is_route_v<_RT_Src>, bool> = true>
        void _inter_route_append_internal(
            const std::shared_ptr<ST> solution,
            const std::function<bool(const std::shared_ptr<ST>)> &aspiration_criteria,
            const std::shared_ptr<ParentInfo<ST>> parent,
            std::shared_ptr<ST> &result,
            std::pair<std::size_t, std::size_t> &tabu_pair,
            std::vector<std::vector<TruckRoute>> &truck_routes,
            std::vector<std::vector<DroneRoute>> &drone_routes)
        {
            if constexpr (X != 0 && Y != 0)
            {
                return;
            }

            constexpr std::size_t Z = X + Y;

            auto problem = Problem::get_instance();

            auto &vehicle_routes_src = utils::match_type<std::vector<std::vector<_RT_Src>>>(truck_routes, drone_routes);
            auto &original_vehicle_routes_src = utils::match_type<std::vector<std::vector<_RT_Src>>>(solution->truck_routes, solution->drone_routes);

            for (std::size_t vehicle_src = 0; vehicle_src < original_vehicle_routes_src.size(); vehicle_src++)
            {
                for (std::size_t route_src = 0; route_src < original_vehicle_routes_src[vehicle_src].size(); route_src++)
                {
                    for (std::size_t vehicle_dest = 0; vehicle_dest < problem->trucks_count + problem->drones_count; vehicle_dest++)
                    {
                        if (vehicle_dest < problem->trucks_count)
                        {
                            if (!solution->truck_routes[vehicle_dest].empty())
                            {
                                continue;
                            }
                        }
                        else
                        {
                            if constexpr (Z != 1)
                            {
                                continue;
                            }
                        }

                        const auto &customers = original_vehicle_routes_src[vehicle_src][route_src].customers();
                        for (std::size_t i = 1; i + Z < customers.size(); i++)
                        {
                            /* Append [i, i + Z) from route_src to vehicle_dest */
                            std::vector<std::size_t> new_customers(customers.begin(), customers.begin() + i);
                            new_customers.insert(new_customers.end(), customers.begin() + (i + Z), customers.end());

                            std::vector<std::size_t> detached = {0};
                            detached.insert(detached.end(), customers.begin() + i, customers.begin() + (i + Z));
                            detached.push_back(0);

                            if constexpr (std::is_same_v<_RT_Src, TruckRoute>)
                            {
                                if (vehicle_dest >= problem->trucks_count &&
                                    std::any_of(
                                        detached.begin() + 1, detached.end() - 1, /* excluding depot */
                                        [&problem](const std::size_t &c)
                                        { return !problem->customers[c].dronable; }))
                                {
                                    continue;
                                }
                            }

                            /* Temporary modify */
                            if (new_customers.size() == 2) // route_src is now empty, check for no-op moves
                            {
                                if constexpr (std::is_same_v<_RT_Src, TruckRoute>)
                                {
                                    if (vehicle_dest < problem->trucks_count && vehicle_src == vehicle_dest)
                                    {
                                        // Same truck
                                        continue;
                                    }
                                }
                                else
                                {
                                    if (vehicle_dest >= problem->trucks_count && vehicle_src == vehicle_dest - problem->trucks_count)
                                    {
                                        // Same drone
                                        continue;
                                    }
                                }

                                vehicle_routes_src[vehicle_src].erase(vehicle_routes_src[vehicle_src].begin() + route_src);
                            }
                            else
                            {
                                vehicle_routes_src[vehicle_src][route_src] = _RT_Src(new_customers);
                            }

                            if (vehicle_dest < problem->trucks_count)
                            {
                                truck_routes[vehicle_dest].push_back(TruckRoute(detached));
                            }
                            else
                            {
                                drone_routes[vehicle_dest - problem->trucks_count].push_back(DroneRoute(detached));
                            }

                            auto new_solution = this->construct(parent, truck_routes, drone_routes);
                            if ((aspiration_criteria(new_solution) || !this->is_tabu(customers[i], static_cast<std::size_t>(0))) &&
                                (result == nullptr || new_solution->cost() < result->cost()))
                            {
                                result = new_solution;
                                tabu_pair = std::make_pair(customers[i], 0);
                            }

                            /* Restore */
                            if (new_customers.size() == 2)
                            {
                                vehicle_routes_src[vehicle_src].insert(
                                    vehicle_routes_src[vehicle_src].begin() + route_src,
                                    original_vehicle_routes_src[vehicle_src][route_src]);
                            }
                            else
                            {
                                vehicle_routes_src[vehicle_src][route_src] = original_vehicle_routes_src[vehicle_src][route_src];
                            }
                            if (vehicle_dest < problem->trucks_count)
                            {
                                truck_routes[vehicle_dest].pop_back();
                            }
                            else
                            {
                                drone_routes[vehicle_dest - problem->trucks_count].pop_back();
                            }
                        }
                    }
                }
            }
        }

    public:
        std::string label() const override
        {
            return utils::format("Move (%d, %d)", X, Y);
        }

        std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> inter_route(
            const std::shared_ptr<ST> solution,
            const std::function<bool(const std::shared_ptr<ST>)> &aspiration_criteria) override
        {
            auto problem = Problem::get_instance();
            auto parent = this->parent_ptr(solution);

            std::shared_ptr<ST> result;
            std::pair<std::size_t, std::size_t> tabu_pair;

            std::vector<std::vector<TruckRoute>> truck_routes(solution->truck_routes);
            std::vector<std::vector<DroneRoute>> drone_routes(solution->drone_routes);

            for (std::size_t vehicle_i = 0; vehicle_i < problem->trucks_count + problem->drones_count; vehicle_i++)
            {
                for (std::size_t vehicle_j = (X == Y ? vehicle_i : 0); vehicle_j < problem->trucks_count + problem->drones_count; vehicle_j++)
                {
                    if (vehicle_i < problem->trucks_count)
                    {
                        if (vehicle_j < problem->trucks_count)
                        {
                            _inter_route_internal<TruckRoute, TruckRoute>(solution, aspiration_criteria, parent, result, tabu_pair, truck_routes, drone_routes, vehicle_i, vehicle_j);
                        }
                        else
                        {
                            _inter_route_internal<TruckRoute, DroneRoute>(solution, aspiration_criteria, parent, result, tabu_pair, truck_routes, drone_routes, vehicle_i, vehicle_j);
                        }
                    }
                    else
                    {
                        if (vehicle_j < problem->trucks_count)
                        {
                            _inter_route_internal<DroneRoute, TruckRoute>(solution, aspiration_criteria, parent, result, tabu_pair, truck_routes, drone_routes, vehicle_i, vehicle_j);
                        }
                        else
                        {
                            _inter_route_internal<DroneRoute, DroneRoute>(solution, aspiration_criteria, parent, result, tabu_pair, truck_routes, drone_routes, vehicle_i, vehicle_j);
                        }
                    }
                }
            }

            if constexpr (X == 0 || Y == 0)
            {
                _inter_route_append_internal<TruckRoute>(solution, aspiration_criteria, parent, result, tabu_pair, truck_routes, drone_routes);
                _inter_route_append_internal<DroneRoute>(solution, aspiration_criteria, parent, result, tabu_pair, truck_routes, drone_routes);
            }

            return std::make_pair(result, tabu_pair);
        }
    };

    template <typename ST, std::size_t X, std::size_t Y>
    class MoveXY : public _BaseMoveXY<ST, X, Y>
    {
    private:
        template <typename _RT, std::enable_if_t<is_route_v<_RT>, bool> = true>
        void _intra_route_internal(
            const std::shared_ptr<ST> solution,
            const std::function<bool(const std::shared_ptr<ST>)> &aspiration_criteria,
            const std::shared_ptr<ParentInfo<ST>> parent,
            std::shared_ptr<ST> &result,
            std::pair<std::size_t, std::size_t> &tabu_pair,
            std::vector<std::vector<TruckRoute>> &truck_routes,
            std::vector<std::vector<DroneRoute>> &drone_routes,
            const std::size_t &_X,
            const std::size_t &_Y)
        {
            auto problem = Problem::get_instance();

            auto vehicles_count = utils::ternary<std::is_same_v<_RT, TruckRoute>>(problem->trucks_count, problem->drones_count);
            auto &vehicle_routes = utils::match_type<std::vector<std::vector<_RT>>>(truck_routes, drone_routes);
            auto &original_vehicle_routes = utils::match_type<std::vector<std::vector<_RT>>>(solution->truck_routes, solution->drone_routes);

            for (std::size_t index = 0; index < vehicles_count; index++)
            {
                for (std::size_t route = 0; route < original_vehicle_routes[index].size(); route++)
                {
                    const auto &customers = original_vehicle_routes[index][route].customers();
                    for (std::size_t i = 1; i + 1 < customers.size(); i++)
                    {
                        for (std::size_t j = i + _X; j + _Y < customers.size(); j++)
                        {
                            /* Swap [i, i + _X) and [j, j + _Y) */
                            std::vector<std::size_t> new_customers(customers);
                            if (_X > _Y)
                            {
                                std::swap_ranges(new_customers.begin() + i, new_customers.begin() + i + _Y, new_customers.begin() + j);
                                std::rotate(new_customers.begin() + i + _Y, new_customers.begin() + i + _X, new_customers.begin() + j + _Y);
                            }
                            else
                            {
                                std::swap_ranges(new_customers.begin() + i, new_customers.begin() + i + _X, new_customers.begin() + j);
                                std::rotate(new_customers.begin() + i + _X, new_customers.begin() + j + _X, new_customers.begin() + j + _Y);
                            }

                            /* Temporary modify */
                            vehicle_routes[index][route] = _RT(new_customers);

                            auto new_solution = this->construct(parent, truck_routes, drone_routes);
                            if ((aspiration_criteria(new_solution) || !this->is_tabu(customers[i], customers[j])) &&
                                (result == nullptr || new_solution->cost() < result->cost()))
                            {
                                result = new_solution;
                                tabu_pair = std::make_pair(customers[i], customers[j]);
                            }

                            /* Restore */
                            vehicle_routes[index][route] = original_vehicle_routes[index][route];
                        }
                    }
                }
            }
        }

    protected:
        std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> intra_route(
            const std::shared_ptr<ST> solution,
            const std::function<bool(const std::shared_ptr<ST>)> &aspiration_criteria) override
        {
            auto parent = this->parent_ptr(solution);

            std::shared_ptr<ST> result;
            std::pair<std::size_t, std::size_t> tabu_pair;

            std::vector<std::vector<TruckRoute>> truck_routes(solution->truck_routes);
            std::vector<std::vector<DroneRoute>> drone_routes(solution->drone_routes);

            _intra_route_internal<TruckRoute>(solution, aspiration_criteria, parent, result, tabu_pair, truck_routes, drone_routes, X, Y);
            _intra_route_internal<DroneRoute>(solution, aspiration_criteria, parent, result, tabu_pair, truck_routes, drone_routes, X, Y);
            if constexpr (X != Y)
            {
                _intra_route_internal<TruckRoute>(solution, aspiration_criteria, parent, result, tabu_pair, truck_routes, drone_routes, Y, X);
                _intra_route_internal<DroneRoute>(solution, aspiration_criteria, parent, result, tabu_pair, truck_routes, drone_routes, Y, X);
            }

            return std::make_pair(result, tabu_pair);
        }
    };

    template <typename ST, std::size_t X>
    class MoveXY<ST, X, 0> : public _BaseMoveXY<ST, X, 0>
    {
    private:
        template <typename _RT, std::enable_if_t<is_route_v<_RT>, bool> = true>
        void _intra_route_internal(
            const std::shared_ptr<ST> solution,
            const std::function<bool(const std::shared_ptr<ST>)> &aspiration_criteria,
            const std::shared_ptr<ParentInfo<ST>> parent,
            std::shared_ptr<ST> &result,
            std::pair<std::size_t, std::size_t> &tabu_pair,
            std::vector<std::vector<TruckRoute>> &truck_routes,
            std::vector<std::vector<DroneRoute>> &drone_routes)
        {
            auto problem = Problem::get_instance();

            auto vehicles_count = utils::ternary<std::is_same_v<_RT, TruckRoute>>(problem->trucks_count, problem->drones_count);
            auto &vehicle_routes = utils::match_type<std::vector<std::vector<_RT>>>(truck_routes, drone_routes);
            auto &original_vehicle_routes = utils::match_type<std::vector<std::vector<_RT>>>(solution->truck_routes, solution->drone_routes);

            for (std::size_t index = 0; index < vehicles_count; index++)
            {
                for (std::size_t route = 0; route < original_vehicle_routes[index].size(); route++)
                {
                    const auto &customers = original_vehicle_routes[index][route].customers();
                    for (std::size_t i = 1; i + X < customers.size(); i++)
                    {
                        for (std::size_t j = 1; j < i; j++)
                        {
                            /* Move [i, i + X) to position j (customers[j] = customers[i]) */
                            std::vector<std::size_t> new_customers(customers);
                            std::rotate(new_customers.begin() + j, new_customers.begin() + i, new_customers.begin() + (i + X));

                            vehicle_routes[index][route] = _RT(new_customers);

                            auto new_solution = this->construct(parent, truck_routes, drone_routes);
                            if ((aspiration_criteria(new_solution) || !this->is_tabu(customers[i], customers[j])) &&
                                (result == nullptr || new_solution->cost() < result->cost()))
                            {
                                result = new_solution;
                                tabu_pair = std::make_pair(customers[i], customers[j]);
                            }

                            /* Restore */
                            vehicle_routes[index][route] = original_vehicle_routes[index][route];
                        }

                        for (std::size_t j = i + X; j + 1 < customers.size(); j++)
                        {
                            /* Move [i, i + X) to position j (customers[j] = customers[i]) */
                            std::vector<std::size_t> new_customers(customers);
                            std::rotate(new_customers.begin() + i, new_customers.begin() + (i + X), new_customers.begin() + (j + 1));

                            vehicle_routes[index][route] = _RT(new_customers);

                            auto new_solution = this->construct(parent, truck_routes, drone_routes);
                            if ((aspiration_criteria(new_solution) || !this->is_tabu(customers[i], customers[j])) &&
                                (result == nullptr || new_solution->cost() < result->cost()))
                            {
                                result = new_solution;
                                tabu_pair = std::make_pair(customers[i], customers[j]);
                            }

                            /* Restore */
                            vehicle_routes[index][route] = original_vehicle_routes[index][route];
                        }
                    }
                }
            }
        }

    protected:
        std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> intra_route(
            const std::shared_ptr<ST> solution,
            const std::function<bool(const std::shared_ptr<ST>)> &aspiration_criteria) override
        {
            auto parent = this->parent_ptr(solution);

            std::shared_ptr<ST> result;
            std::pair<std::size_t, std::size_t> tabu_pair;

            std::vector<std::vector<TruckRoute>> truck_routes(solution->truck_routes);
            std::vector<std::vector<DroneRoute>> drone_routes(solution->drone_routes);

            _intra_route_internal<TruckRoute>(solution, aspiration_criteria, parent, result, tabu_pair, truck_routes, drone_routes);
            _intra_route_internal<DroneRoute>(solution, aspiration_criteria, parent, result, tabu_pair, truck_routes, drone_routes);

            return std::make_pair(result, tabu_pair);
        }
    };
}
