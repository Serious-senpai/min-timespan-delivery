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

    template <typename ST, typename RT, std::enable_if_t<is_route_v<RT>, bool> = true>
    bool _try_insert(
        RT &route,
        const std::size_t &customer,
        std::vector<std::vector<TruckRoute>> &truck_routes,
        std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        RT old = route;
        route.push_back(customer);

        if (_insertable<ST>(truck_routes, drone_routes))
        {
            return true;
        }

        route = old;
        return false;
    }

    template <typename ST, typename RT, std::enable_if_t<is_route_v<RT>, bool> = true>
    bool _try_insert(
        std::vector<RT> &routes,
        const std::size_t &customer,
        std::vector<std::vector<TruckRoute>> &truck_routes,
        std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        routes.emplace_back(std::vector<std::size_t>{0, customer, 0});

        if (_insertable<ST>(truck_routes, drone_routes))
        {
            return true;
        }

        routes.pop_back();
        return false;
    }

    template <typename ST>
    void _insert_leftover(
        const std::vector<std::size_t> &leftover,
        std::vector<std::vector<TruckRoute>> &truck_routes,
        const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        auto problem = Problem::get_instance();

        std::vector<std::size_t> leftover_copy(leftover);
        std::shuffle(leftover_copy.begin(), leftover_copy.end(), utils::rng);

        for (auto &customer : leftover_copy)
        {
            std::vector<std::vector<TruckRoute>> truck_routes_modified(truck_routes);
            std::shared_ptr<ST> temp = nullptr;

            for (std::size_t truck = 0; truck < problem->trucks_count; truck++)
            {
                for (std::size_t route = 0; route < truck_routes[truck].size(); route++)
                {
                    const auto &customers = truck_routes[truck][route].customers();
                    for (std::size_t i = 1; i < customers.size(); i++)
                    {
                        std::vector<std::size_t> r(customers);
                        r.insert(r.begin() + i, customer);

                        // Temporary modify
                        truck_routes_modified[truck][route] = TruckRoute(r);

                        auto new_temp = std::make_shared<ST>(truck_routes_modified, drone_routes, nullptr, false);
                        if (temp == nullptr || new_temp->working_time < temp->working_time || (new_temp->feasible && !temp->feasible))
                        {
                            temp = new_temp;
                        }

                        // Restore
                        truck_routes_modified[truck][route] = truck_routes[truck][route];
                    }
                }

                // Temporary append
                truck_routes_modified[truck].emplace_back(std::vector<std::size_t>{0, customer, 0});

                auto new_temp = std::make_shared<ST>(truck_routes_modified, drone_routes, nullptr, false);
                if (temp == nullptr || new_temp->working_time < temp->working_time || (new_temp->feasible && !temp->feasible))
                {
                    temp = new_temp;
                }

                // Restore
                truck_routes_modified[truck].pop_back();
            }

            truck_routes = temp->truck_routes;
        }
    }

    std::vector<std::vector<std::size_t>> clusterize_1(
        const std::vector<std::size_t> &customers,
        const std::size_t &k)
    {
        std::vector<std::vector<std::size_t>> clusters(k);
        if (customers.empty())
        {
            return clusters;
        }

        auto problem = Problem::get_instance();

        alglib::clusterizerstate state;
        alglib::clusterizercreate(state);

        alglib::real_2d_array matrix;
        std::vector<double> xy;
        xy.reserve(2 * customers.size());
        for (auto &index : customers)
        {
            xy.push_back(problem->customers[index].x);
            xy.push_back(problem->customers[index].y);
        }
        matrix.setcontent(customers.size(), 2, xy.data());

        alglib::clusterizersetpoints(state, matrix, 2);
        alglib::clusterizersetkmeanslimits(state, 1, 500);

        alglib::kmeansreport report;
        alglib::clusterizerrunkmeans(state, k, report);

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
                clusters[i % k].push_back(customers[i]);
            }
        }

        return clusters;
    }

    std::vector<std::vector<std::size_t>> clusterize_2(
        const std::vector<std::size_t> &customers,
        const std::size_t &k)
    {
        std::vector<std::vector<std::size_t>> clusters(k);
        if (customers.empty())
        {
            return clusters;
        }

        auto problem = Problem::get_instance();
        std::vector<double> angles(customers.size());
        std::transform(
            customers.begin(), customers.end(), angles.begin(),
            [&problem](const std::size_t &customer)
            {
                double a = std::atan2(
                    problem->customers[customer].y - problem->customers[0].y,
                    problem->customers[customer].x - problem->customers[0].x);
                if (a < 0)
                {
                    a += 2 * M_PI;
                }

                return a; // range [0, 2 * M_PI]
            });

        std::vector<std::size_t> ordered(customers.size());

        {
            std::vector<std::size_t> index(customers.size());
            std::iota(index.begin(), index.end(), 0);

            std::sort(
                index.begin(), index.end(),
                [&angles](const std::size_t &first, const std::size_t &second)
                {
                    return angles[first] < angles[second];
                });

            std::transform(
                index.begin(), index.end(), ordered.begin(),
                [&customers](const std::size_t &i)
                {
                    return customers[i];
                });
        }

        std::sort(angles.begin(), angles.end());

        const auto angle_diff = [](const double &from, const double &to)
        {
            double diff = to - from;
            return diff < 0 ? diff + 2 * M_PI : diff;
        };

        {
            std::size_t shift = 0;
            double max_diff = -1;
            for (std::size_t i = 0; i < angles.size(); i++)
            {
                double diff = angle_diff(angles[(i + angles.size() - 1) % angles.size()], angles[i]);
                if (diff < 0)
                {
                    diff += 2 * M_PI;
                }

                if (diff > max_diff)
                {
                    shift = i;
                    max_diff = diff;
                }
            }

            std::rotate(ordered.begin(), ordered.begin() + shift, ordered.end());
            std::rotate(angles.begin(), angles.begin() + shift, angles.end());
        }

        {
            const double base = angles.front();
            std::transform(
                angles.begin(), angles.end(), angles.begin(),
                [&angle_diff, &base](const double &angle)
                {
                    return angle_diff(base, angle);
                });
        }

        double angle_shift = (angles.back() - angles.front()) / static_cast<double>(k);
        for (std::size_t i = 0; i < angles.size(); i++)
        {
            auto cidx = std::min<std::size_t>(std::floor(angles[i] / angle_shift), k - 1);
            clusters[cidx].push_back(ordered[i]);
        }

        return clusters;
    }

    void _sort_cluster_with_starting_point(std::vector<std::size_t> &cluster, const std::size_t &start)
    {
        auto problem = Problem::get_instance();
        cluster.insert(cluster.begin(), start); // So that nearest_heuristic can start from starting point

        const auto distance = [&problem, &cluster](const std::size_t &i, const std::size_t &j)
        {
            return problem->distances[cluster[i]][cluster[j]];
        };

        auto [_, order] = cluster.size() < 20
                              ? utils::held_karp_algorithm(cluster.size(), distance)
                              : utils::nearest_heuristic(cluster.size(), distance);

        std::transform(
            order.begin(), order.end(), order.begin(),
            [&cluster](const std::size_t &i)
            {
                return cluster[i];
            });

        std::rotate(order.begin(), std::find(order.begin(), order.end(), start), order.end());
        std::reverse(order.begin(), order.end());
        order.pop_back();

        cluster = order;
    }

    template <typename ST, int _Clusterizer, typename RT, std::enable_if_t<is_route_v<RT> && (_Clusterizer == 1 || _Clusterizer == 2), bool> = true>
    std::vector<std::vector<RT>> _initial_helper(const std::vector<std::size_t> &customers, std::vector<std::size_t> *unhandled_ptr)
    {
        auto problem = Problem::get_instance();
        std::vector<std::vector<TruckRoute>> truck_routes(problem->trucks_count);
        std::vector<std::vector<DroneRoute>> drone_routes(problem->drones_count);

        auto vehicles_count = utils::ternary<std::is_same_v<RT, TruckRoute>>(problem->trucks_count, problem->drones_count);
        auto &vehicle_routes = utils::match_type<std::vector<std::vector<RT>>>(truck_routes, drone_routes);

        std::vector<std::vector<std::size_t>> clusters;
        if constexpr (_Clusterizer == 1)
        {
            clusters = clusterize_1(customers, vehicles_count);
        }
        else
        {
            clusters = clusterize_2(customers, vehicles_count);
        }

        std::iter_swap(clusters.begin(), utils::random_element(clusters));

        bool new_route;
        std::vector<std::size_t> vehicle(vehicles_count);
        std::iota(vehicle.begin(), vehicle.end(), 0);

        const auto select_vehicle = [&truck_routes, &drone_routes, &new_route, &vehicle]()
        {
            auto temp = std::make_shared<ST>(truck_routes, drone_routes, nullptr, false);
            std::vector<double> working_time = utils::ternary<std::is_same_v<RT, TruckRoute>>(temp->truck_working_time, temp->drone_working_time);

            new_route = true;
            std::sort(
                vehicle.begin(), vehicle.end(),
                [&working_time](const std::size_t &i, const std::size_t &j)
                {
                    return working_time[i] < working_time[j];
                });
        };

        select_vehicle();
        std::size_t cluster_i = 0, last_customer = std::size_t(-1);

        while (std::any_of(clusters.begin(), clusters.end(), [](const std::vector<std::size_t> &c)
                           { return !c.empty(); }))
        {
            // Loop invariant: clusters[cluster_i] may be empty, inserting to vehicle route may yield infeasible route
            if (clusters[cluster_i].empty())
            {
                std::vector<double> distances(clusters.size(), std::numeric_limits<double>::max());
                for (std::size_t i = 0; i < clusters.size(); i++)
                {
                    for (auto &customer : clusters[i])
                    {
                        distances[i] = std::min(distances[i], problem->distances[last_customer][customer]);
                    }
                }
                cluster_i = std::min_element(distances.begin(), distances.end()) - distances.begin();

                _sort_cluster_with_starting_point(clusters[cluster_i], last_customer);
                continue;
            }

            // clusters[cluster_i] is guaranteed to be non-empty now
            if (new_route)
            {
                // Reorder customers in clusters[cluster_i]
                _sort_cluster_with_starting_point(clusters[cluster_i], 0);

                // Construct new vehicle route
                if (!_try_insert<ST>(vehicle_routes[vehicle[0]], clusters[cluster_i].back(), truck_routes, drone_routes))
                {
                    bool inserted = false;
                    for (std::size_t i = 1; i < vehicles_count; i++)
                    {
                        if (_try_insert<ST>(vehicle_routes[vehicle[i]], clusters[cluster_i].back(), truck_routes, drone_routes))
                        {
                            last_customer = clusters[cluster_i].back();
                            clusters[cluster_i].pop_back();
                            new_route = false;

                            std::rotate(vehicle.begin(), vehicle.begin() + i, vehicle.end());

                            inserted = true;
                            break;
                        }
                    }

                    if (!inserted)
                    {
                        for (std::size_t i = 0; i < clusters.size(); i++)
                        {
                            if (!clusters[i].empty())
                            {
                                _sort_cluster_with_starting_point(clusters[i], 0);
                                if (_try_insert<ST>(vehicle_routes[vehicle[0]], clusters[i].back(), truck_routes, drone_routes))
                                {
                                    cluster_i = i;

                                    last_customer = clusters[cluster_i].back();
                                    clusters[cluster_i].pop_back();
                                    new_route = false;

                                    inserted = true;
                                    break;
                                }
                            }
                        }
                    }

                    if (!inserted)
                    {
                        break;
                    }
                }
                else
                {
                    last_customer = clusters[cluster_i].back();
                    clusters[cluster_i].pop_back();
                    new_route = false;
                }
            }
            else
            {
                // Insert customer to last route of vehicle
                if (!_try_insert<ST>(vehicle_routes[vehicle[0]].back(), clusters[cluster_i].back(), truck_routes, drone_routes))
                {
                    select_vehicle();
                }
                else
                {
                    last_customer = clusters[cluster_i].back();
                    clusters[cluster_i].pop_back();
                }
            }
        }

        if (unhandled_ptr != nullptr)
        {
            for (auto &cluster : clusters)
            {
                unhandled_ptr->insert(unhandled_ptr->end(), cluster.begin(), cluster.end());
            }
        }

        return vehicle_routes;
    }

    template <typename ST, int _Clusterizer>
    std::shared_ptr<ST> initial_impl()
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

        std::vector<std::vector<DroneRoute>> drone_routes = _initial_helper<ST, _Clusterizer, DroneRoute>(dronable, &truck_only);

        std::vector<std::size_t> leftover;
        std::vector<std::vector<TruckRoute>> truck_routes = _initial_helper<ST, _Clusterizer, TruckRoute>(truck_only, &leftover);

        _insert_leftover<ST>(leftover, truck_routes, drone_routes);
        return std::make_shared<ST>(truck_routes, drone_routes, std::make_shared<ParentInfo<ST>>(nullptr, utils::format("initial-%d", _Clusterizer)));
    }
}
