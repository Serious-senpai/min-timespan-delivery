#pragma once

#include "parent.hpp"
#include "random.hpp"
#include "routes.hpp"

namespace d2d
{
    template <typename RT, typename ST, std::enable_if_t<std::disjunction_v<std::is_same<RT, TruckRoute>, std::is_same<RT, DroneRoute>>, bool> = true>
    bool _try_insert(
        RT &route,
        const std::size_t &customer,
        std::vector<std::vector<TruckRoute>> &truck_routes,
        std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        RT old = route;
        route.push_back(customer);

        auto solution = std::make_shared<ST>(truck_routes, drone_routes, nullptr, false);
        if (solution->feasible)
        {
            return true;
        }

        route = old;
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
                drone_iter->push_back(DroneRoute({0, customer, 0}));
                drone_iter++;
            }
            else if (truck_iter != truck_routes.end())
            {
                truck_iter->push_back(TruckRoute({0, customer, 0}));
                truck_iter++;
            }
            else
            {
                next_phase.push_back(customer);
            }
        }

        std::size_t truck = 0, drone = 0;
        std::shuffle(next_phase.begin(), next_phase.end(), utils::rng);
        while (!next_phase.empty())
        {
            auto customer = next_phase.back();
            next_phase.pop_back();
            if (problem->customers[customer].dronable)
            {
                if (!_try_insert<DroneRoute, ST>(drone_routes[drone % problem->drones_count].back(), customer, truck_routes, drone_routes))
                {
                    drone_routes[drone % problem->drones_count].push_back(DroneRoute({0, customer, 0}));
                }
                drone++;
            }
            else
            {
                if (!_try_insert<TruckRoute, ST>(truck_routes[truck % problem->trucks_count].back(), customer, truck_routes, drone_routes))
                {
                    truck_routes[truck % problem->trucks_count].push_back(TruckRoute({0, customer, 0}));
                }
                truck++;
            }
        }
        return std::make_shared<ST>(truck_routes, drone_routes, nullptr);
    }

    template <typename RT, typename ST, std::enable_if_t<std::disjunction_v<std::is_same<RT, TruckRoute>, std::is_same<RT, DroneRoute>>, bool> = true>
    std::vector<std::vector<RT>> _initial_2_helper(const std::vector<std::size_t> &customers)
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
        for (std::size_t i = 0; i < customers.size(); i++)
        {
            clusters[report.cidx[i]].push_back(customers[i]);
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
                                return problem->distances[cluster[i]][cluster[j]];
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
                            return problem->distances[cluster[*iter]][cluster[i]] < problem->distances[cluster[*iter]][cluster[j]];
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

        std::vector<double> working_time(vehicles_count);
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

            std::size_t vehicle = std::min_element(working_time.begin(), working_time.end()) - working_time.begin();

            if constexpr (std::is_same_v<RT, TruckRoute>)
            {
                truck_routes[vehicle].push_back(TruckRoute({0, clusters[cluster_i].back(), 0}));
            }
            else
            {
                drone_routes[vehicle].push_back(DroneRoute({0, clusters[cluster_i].back(), 0}));
            }

            clusters[cluster_i].pop_back();

            while (!clusters[cluster_i].empty())
            {
                bool inserted = false;
                if constexpr (std::is_same_v<RT, TruckRoute>)
                {
                    inserted = _try_insert<TruckRoute, ST>(truck_routes[vehicle].back(), clusters[cluster_i].back(), truck_routes, drone_routes);
                }
                else
                {
                    inserted = _try_insert<DroneRoute, ST>(drone_routes[vehicle].back(), clusters[cluster_i].back(), truck_routes, drone_routes);
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

        std::vector<std::vector<TruckRoute>> truck_routes = _initial_2_helper<TruckRoute, ST>(truck_only);
        std::vector<std::vector<DroneRoute>> drone_routes = _initial_2_helper<DroneRoute, ST>(dronable);

        return std::make_shared<ST>(truck_routes, drone_routes, nullptr);
    }
}
