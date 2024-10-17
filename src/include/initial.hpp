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
                // std::cerr << "Angle of " << ordered[i] << " = " << angles[i] * 180 / M_PI << std::endl;
                // std::cerr << "Angle from " << ordered[(i + angles.size() - 1) % angles.size()] << " to " << ordered[i] << " = " << diff * 180 / M_PI << std::endl;
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
            // std::cerr << "Shifted to customer " << shift << " where max_diff = " << max_diff << std::endl;
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

    template <typename ST, int _Clusterizer>
    std::shared_ptr<ST> initial_impl()
    {
        auto problem = Problem::get_instance();

        std::vector<std::size_t> index(problem->customers.size() - 1);
        std::iota(index.begin(), index.end(), 1);

        std::vector<std::vector<std::size_t>> clusters;
        if constexpr (_Clusterizer == 1)
        {
            clusters = clusterize_1(index, problem->trucks_count);
        }
        else
        {
            clusters = clusterize_2(index, problem->trucks_count);
        }

        // std::cerr << clusters << std::endl;
        std::vector<std::vector<TruckRoute>> truck_routes(problem->trucks_count);
        std::vector<std::vector<DroneRoute>> drone_routes(problem->trucks_count); // Will resize to problem->drones_count later

        std::size_t truck = 0, drone = 0;
        std::set<std::pair<double, std::pair<std::size_t, bool>>> timestamps;
        for (std::size_t i = 0; i < clusters.size(); i++)
        {
            // std::cerr << "At cluster " << i << ": " << clusters[i] << std::endl;
            std::sort(
                clusters[i].begin(), clusters[i].end(), [&problem](const std::size_t &i, const std::size_t &j)
                { return problem->distances[0][i] < problem->distances[0][j]; });
            // std::cerr << "Sorted to " << clusters[i] << std::endl;

            auto truck_customer = clusters[i].back(), drone_customer = clusters[i].front();
            auto temp = std::make_shared<ST>(truck_routes, drone_routes, nullptr, false);
            timestamps.insert(std::make_pair(temp->truck_working_time[truck], std::make_pair(truck_customer, true)));
            timestamps.insert(std::make_pair(temp->drone_working_time[drone], std::make_pair(drone_customer, false)));

            while (!clusters[i].empty())
            {
                auto packed = timestamps.begin();
                // std::cerr << "timestamps = " << timestamps << std::endl;
                // std::cerr << "clusters[" << i << "] = " << clusters[i] << std::endl;
                // std::cerr << "packed = " << *packed << std::endl;
                timestamps.erase(timestamps.begin());

                auto [customer, is_truck] = packed->second;
                if (is_truck)
                {
                    auto nearest = std::min_element(
                        clusters[i].begin(), clusters[i].end(),
                        [&problem, &customer](const std::size_t &i, const std::size_t &j)
                        {
                            return problem->distances[customer][i] < problem->distances[customer][j];
                        });

                    auto temp = std::make_shared<ST>(truck_routes, drone_routes, nullptr, false);
                    timestamps.insert(std::make_pair(temp->truck_working_time[truck], std::make_pair(*nearest, true)));
                }
                else
                {
                    std::vector<std::size_t> dronable(clusters[i].size());
                    auto end = std::remove_copy_if(
                        clusters[i].begin(), clusters[i].end(), dronable.begin(),
                        [&problem](const std::size_t &i)
                        {
                            return !problem->customers[i].dronable;
                        });
                    auto nearest = std::min_element(
                        dronable.begin(), end,
                        [&problem, &customer](const std::size_t &i, const std::size_t &j)
                        {
                            return problem->distances[customer][i] < problem->distances[customer][j];
                        });

                    if (nearest != end) // No dronable customers
                    {
                        auto temp = std::make_shared<ST>(truck_routes, drone_routes, nullptr, false);
                        timestamps.insert(std::make_pair(temp->drone_working_time[drone], std::make_pair(*nearest, false)));
                    }
                }

                auto iter = std::find(clusters[i].begin(), clusters[i].end(), customer);
                if (iter == clusters[i].end())
                {
                    continue;
                }

                clusters[i].erase(iter);

                if (is_truck)
                {
                    // std::cerr << "Inserting " << customer << " to truck routes " << truck_routes[truck] << std::endl;
                    bool insertable;
                    if (truck_routes[truck].empty())
                    {
                        insertable = _try_insert<ST>(truck_routes[truck], customer, truck_routes, drone_routes);
                    }
                    else
                    {
                        insertable = _try_insert<ST>(truck_routes[truck].back(), customer, truck_routes, drone_routes);
                    }

                    if (!insertable)
                    {
                        auto temp = std::make_shared<ST>(truck_routes, drone_routes, nullptr, false);
                        truck = std::distance(
                            temp->truck_working_time.begin(),
                            std::min_element(temp->truck_working_time.begin(), temp->truck_working_time.end()));
                        // std::cerr << "Moving to truck " << truck << ", current routes = " << truck_routes[truck] << std::endl;

                        truck_routes[truck].emplace_back(std::vector<std::size_t>{0, customer, 0});
                    }
                }
                else
                {
                    // std::cerr << "Inserting " << customer << " to drone routes " << drone_routes[drone] << std::endl;

                    bool insertable;
                    if (drone_routes[drone].empty())
                    {
                        insertable = _try_insert<ST>(drone_routes[drone], customer, truck_routes, drone_routes);
                    }
                    else
                    {
                        insertable = _try_insert<ST>(drone_routes[drone].back(), customer, truck_routes, drone_routes);
                    }

                    if (!insertable)
                    {
                        auto temp = std::make_shared<ST>(truck_routes, drone_routes, nullptr, false);
                        drone = std::distance(
                            temp->drone_working_time.begin(),
                            std::min_element(temp->drone_working_time.begin(), temp->drone_working_time.end()));
                        // std::cerr << "Moving to drone " << drone << ", current routes = " << drone_routes[drone] << std::endl;

                        drone_routes[drone].emplace_back(std::vector<std::size_t>{0, customer, 0});
                    }
                }

                // auto temp = std::make_shared<ST>(truck_routes, drone_routes, nullptr, false);
                // std::cerr << "truck_routes = " << truck_routes << " \e[31m" << temp->truck_working_time << "\e[0m" << std::endl;
                // std::cerr << "drone_routes = " << drone_routes << " \e[31m" << temp->drone_working_time << "\e[0m" << std::endl;
                // std::cerr << "feasible = " << temp->feasible << " " << temp->capacity_violation << " " << temp->waiting_time_violation << " " << temp->fixed_time_violation << std::endl;
            }
        }

        // TODO: Resize drone routes to `problem->drones_count`

        return std::make_shared<ST>(truck_routes, drone_routes, std::make_shared<ParentInfo<ST>>(nullptr, utils::format("initial-%d", _Clusterizer)));
    }
}
