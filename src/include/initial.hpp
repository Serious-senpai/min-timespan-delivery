#pragma once

#include "parent.hpp"
#include "random.hpp"
#include "routes.hpp"

namespace d2d
{
    template <typename ST>
    bool _insertable(
        const std::vector<std::vector<TruckRoute>> &truck_routes,
        const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        return std::make_shared<ST>(truck_routes, drone_routes, nullptr, false)->feasible;
    }

    template <typename RT, typename ST, std::enable_if_t<std::disjunction_v<std::is_same<RT, TruckRoute>, std::is_same<RT, DroneRoute>>, bool> = true>
    bool _try_insert(
        RT &route,
        const std::size_t &customer,
        std::vector<std::vector<TruckRoute>> &truck_routes,
        std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        if constexpr (std::is_same_v<RT, DroneRoute>)
        {
            return false;
        }

        RT old = route;
        route.push_back(customer);

        if (_insertable<ST>(truck_routes, drone_routes))
        {
            return true;
        }

        route = old;
        return false;
    }

    template <typename RT, typename ST, std::enable_if_t<std::disjunction_v<std::is_same<RT, TruckRoute>, std::is_same<RT, DroneRoute>>, bool> = true>
    bool _try_insert(
        std::vector<RT> &routes,
        const std::size_t &customer,
        std::vector<std::vector<TruckRoute>> &truck_routes,
        std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        if constexpr (std::is_same_v<RT, TruckRoute>)
        {
            if (routes.size() == 1)
            {
                return false;
            }
        }

        routes.emplace_back(std::vector<std::size_t>{0, customer, 0});

        if (_insertable<ST>(truck_routes, drone_routes))
        {
            return true;
        }

        routes.pop_back();
        return false;
    }

    template <typename ST>
    std::shared_ptr<ST> initial_1()
    {
        auto problem = Problem::get_instance();
        std::vector<std::vector<TruckRoute>> truck_routes(problem->trucks_count);
        std::vector<std::vector<DroneRoute>> drone_routes(problem->drones_count);

        std::vector<std::size_t> customers_by_angle(problem->customers.size() - 1);
        std::iota(customers_by_angle.begin(), customers_by_angle.end(), 1);
        std::sort(
            customers_by_angle.begin(), customers_by_angle.end(),
            [&problem](const std::size_t &first, const std::size_t &second)
            {
                return std::atan2(problem->customers[first].y, problem->customers[first].x) <
                       std::atan2(problem->customers[second].y, problem->customers[second].x);
            });

        std::rotate(
            customers_by_angle.begin(),
            customers_by_angle.begin() + utils::random(static_cast<std::size_t>(0), problem->customers.size() - 1),
            customers_by_angle.end());

        auto truck_iter = truck_routes.begin();
        auto drone_iter = drone_routes.begin();
        std::vector<std::size_t> next_phase;
        for (auto &customer : customers_by_angle)
        {
            if (drone_iter != drone_routes.end() && problem->customers[customer].dronable)
            {
                if (_try_insert<DroneRoute, ST>(*drone_iter, customer, truck_routes, drone_routes))
                {
                    drone_iter++;
                    continue;
                }
            }

            if (truck_iter != truck_routes.end())
            {
                if (_try_insert<TruckRoute, ST>(*truck_iter, customer, truck_routes, drone_routes))
                {
                    truck_iter++;
                    continue;
                }
            }

            next_phase.push_back(customer);
        }

        std::size_t truck = 0, drone = 0;
        for (std::size_t iteration = 0; iteration < utils::pow2(problem->customers.size()) / 2 && !next_phase.empty(); iteration++)
        {
            auto iter = utils::random_element(next_phase);
            auto customer = *iter;
            next_phase.erase(iter);

            if (problem->customers[customer].dronable)
            {
                if ((!drone_routes[drone].empty() && _try_insert<DroneRoute, ST>(drone_routes[drone].back(), customer, truck_routes, drone_routes)) ||
                    _try_insert<DroneRoute, ST>(drone_routes[drone], customer, truck_routes, drone_routes))
                {
                    drone = (drone + 1) % problem->drones_count;
                    continue;
                }
            }

            if ((!truck_routes[truck].empty() && _try_insert<TruckRoute, ST>(truck_routes[truck].back(), customer, truck_routes, drone_routes)) ||
                _try_insert<TruckRoute, ST>(truck_routes[truck], customer, truck_routes, drone_routes))
            {
                truck = (truck + 1) % problem->trucks_count;
                continue;
            }

            next_phase.push_back(customer);
        }

        for (auto &customer : next_phase)
        {
            auto truck = utils::random(static_cast<std::size_t>(0), problem->trucks_count - 1);
            if (truck_routes[truck].empty())
            {
                truck_routes[truck].emplace_back(std::vector<std::size_t>{0, customer, 0});
            }
            else
            {
                truck_routes[truck].back().push_back(customer);
            }
        }

        return std::make_shared<ST>(truck_routes, drone_routes, std::make_shared<ParentInfo<ST>>(nullptr, "initial-1"));
    }

    template <typename RT, typename ST, std::enable_if_t<std::disjunction_v<std::is_same<RT, TruckRoute>, std::is_same<RT, DroneRoute>>, bool> = true>
    std::vector<std::vector<RT>> _initial_2_helper(const std::vector<std::size_t> &customers, std::vector<std::size_t> *unhandled_ptr)
    {
        auto problem = Problem::get_instance();
        std::vector<std::vector<TruckRoute>> truck_routes(problem->trucks_count);
        std::vector<std::vector<DroneRoute>> drone_routes(problem->drones_count);

        std::size_t vehicles_count;
        if constexpr (std::is_same_v<RT, TruckRoute>)
        {
            vehicles_count = problem->trucks_count;
        }
        else
        {
            vehicles_count = problem->drones_count;
        }

        alglib::clusterizerstate state;
        alglib::clusterizercreate(state);

        alglib::real_2d_array matrix;
        std::vector<double> xy;
        xy.reserve(2 * customers.size());
        for (auto index : customers)
        {
            xy.push_back(problem->customers[index].x);
            xy.push_back(problem->customers[index].y);
        }
        matrix.setcontent(customers.size(), 2, xy.data());

        alglib::clusterizersetpoints(state, matrix, 2);
        alglib::clusterizersetkmeanslimits(state, 1, 500);

        alglib::kmeansreport report;
        alglib::clusterizerrunkmeans(state, vehicles_count, report);

        std::vector<std::vector<std::size_t>> clusters(vehicles_count);
        if (report.terminationtype == 1)
        {
            for (std::size_t i = 0; i < customers.size(); i++)
            {
                clusters[report.cidx[i]].push_back(customers[i]);
            }
        }
        else
        {
            for (std::size_t i = 0; i < customers.size(); i++)
            {
                clusters[i % clusters.size()].push_back(customers[i]);
            }
        }

        for (auto &cluster : clusters)
        {
            cluster.push_back(0);

            std::vector<std::size_t> order(cluster.size());
            std::iota(order.begin(), order.end(), 0);
            if (cluster.size() < 20)
            {
                order = utils::held_karp_algorithm(
                            cluster.size(),
                            [&problem, &cluster](const std::size_t &i, const std::size_t &j)
                            {
                                if constexpr (std::is_same_v<RT, TruckRoute>)
                                {
                                    return problem->man_distances[cluster[i]][cluster[j]];
                                }
                                else
                                {
                                    return problem->euc_distances[cluster[i]][cluster[j]];
                                }
                            })
                            .second;
            }
            else
            {
                for (auto iter = order.begin(); iter != order.end(); iter++)
                {
                    auto nearest = std::min_element(
                        iter + 1, order.end(),
                        [&problem, &cluster, &iter](const std::size_t &i, const std::size_t &j)
                        {
                            if constexpr (std::is_same_v<RT, TruckRoute>)
                            {
                                return problem->man_distances[cluster[*iter]][cluster[i]] < problem->man_distances[cluster[*iter]][cluster[j]];
                            }
                            else
                            {
                                return problem->euc_distances[cluster[*iter]][cluster[i]] < problem->euc_distances[cluster[*iter]][cluster[j]];
                            }
                        });

                    if (nearest != order.end())
                    {
                        std::iter_swap(iter + 1, nearest);
                    }
                }
            }

            std::vector<std::size_t> cluster_ordered(cluster.size());
            for (std::size_t i = 0; i < cluster.size(); i++)
            {
                cluster_ordered[i] = cluster[order[i]];
            }

            if (cluster_ordered.back() != 0)
            {
                std::rotate(
                    cluster_ordered.begin(),
                    std::find(cluster_ordered.begin(), cluster_ordered.end(), 0) + 1,
                    cluster_ordered.end());
            }
            cluster_ordered.pop_back();
            cluster = cluster_ordered;
        }

        for (std::size_t cluster_i = 0; cluster_i < clusters.size(); cluster_i = (cluster_i + 1) % clusters.size())
        {
            if (clusters[cluster_i].empty())
            {
                if (std::all_of(clusters.begin(), clusters.end(), [](const std::vector<std::size_t> &cluster)
                                { return cluster.empty(); }))
                {
                    break;
                }

                continue;
            }

            std::size_t vehicle = (std::size_t)-1;
            {
                auto temp = std::make_shared<ST>(truck_routes, drone_routes, nullptr, false);
                if constexpr (std::is_same_v<RT, TruckRoute>)
                {
                    vehicle = std::min_element(
                                  temp->truck_working_time.begin(),
                                  temp->truck_working_time.end(),
                                  [](const double &first, const double &second)
                                  { return first < second; }) -
                              temp->truck_working_time.begin();
                }
                else
                {
                    vehicle = std::min_element(
                                  temp->drone_working_time.begin(),
                                  temp->drone_working_time.end(),
                                  [](const double &first, const double &second)
                                  { return first < second; }) -
                              temp->drone_working_time.begin();
                }
            }

            std::size_t customer = clusters[cluster_i].back();
            clusters[cluster_i].pop_back();

            if constexpr (std::is_same_v<RT, TruckRoute>)
            {
                if (!_try_insert<TruckRoute, ST>(truck_routes[vehicle], customer, truck_routes, drone_routes))
                {
                    if (unhandled_ptr != nullptr)
                    {
                        unhandled_ptr->push_back(customer);
                    }

                    continue;
                }
            }
            else
            {
                if (!_try_insert<DroneRoute, ST>(drone_routes[vehicle], customer, truck_routes, drone_routes))
                {
                    if (unhandled_ptr != nullptr)
                    {
                        unhandled_ptr->push_back(customer);
                    }

                    continue;
                }
            }

            while (!clusters[cluster_i].empty())
            {
                std::size_t customer = clusters[cluster_i].back();
                bool inserted = false;
                if constexpr (std::is_same_v<RT, TruckRoute>)
                {
                    inserted = _try_insert<TruckRoute, ST>(truck_routes[vehicle].back(), customer, truck_routes, drone_routes);
                }
                else
                {
                    inserted = _try_insert<DroneRoute, ST>(drone_routes[vehicle].back(), customer, truck_routes, drone_routes);
                }

                if (inserted)
                {
                    clusters[cluster_i].pop_back();
                }
                else
                {
                    break;
                }
            }
        }

        if constexpr (std::is_same_v<RT, TruckRoute>)
        {
            return truck_routes;
        }
        else
        {
            return drone_routes;
        }
    }

    template <typename ST>
    std::shared_ptr<ST> initial_2()
    {
        auto problem = Problem::get_instance();

        std::vector<std::size_t> dronable, truck_only;
        for (std::size_t i = 1; i < problem->customers.size(); i++)
        {
            if (problem->customers[i].dronable)
            {
                dronable.push_back(i);
            }
            else
            {
                truck_only.push_back(i);
            }
        }

        std::vector<std::vector<DroneRoute>> drone_routes = _initial_2_helper<DroneRoute, ST>(dronable, &truck_only);

        std::vector<std::size_t> left_over;
        std::vector<std::vector<TruckRoute>> truck_routes = _initial_2_helper<TruckRoute, ST>(truck_only, &left_over);

        for (auto &customer : left_over)
        {
            auto truck = utils::random(static_cast<std::size_t>(0), problem->trucks_count - 1);
            if (truck_routes[truck].empty())
            {
                truck_routes[truck].emplace_back(std::vector<std::size_t>{0, customer, 0});
            }
            else
            {
                truck_routes[truck].back().push_back(customer);
            }
        }

        return std::make_shared<ST>(truck_routes, drone_routes, std::make_shared<ParentInfo<ST>>(nullptr, "initial-2"));
    }
}
