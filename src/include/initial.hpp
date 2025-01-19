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
        auto temp = std::make_shared<ST>(truck_routes, drone_routes, nullptr, false);
        if (!temp->feasible)
        {
            // std::cerr << "Insertion would violate " << temp->drone_energy_violation << " " << temp->capacity_violation << " " << temp->waiting_time_violation << " " << temp->fixed_time_violation << std::endl;
            return false;
        }

        return true;
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

    struct _initialization_iteration_pack
    {
        double working_time;
        std::size_t vehicle;
        std::size_t before;
        std::size_t customer;
        bool is_truck;

        _initialization_iteration_pack(
            const double &working_time,
            const std::size_t &vehicle,
            const std::size_t &before,
            const std::size_t &customer,
            const bool &is_truck)
            : working_time(working_time),
              vehicle(vehicle),
              before(before),
              customer(customer),
              is_truck(is_truck) {}

        bool operator<(const _initialization_iteration_pack &other) const
        {
            return working_time < other.working_time;
        }
    };

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

        std::vector<std::size_t> clusters_mapping(problem->customers.size());
        for (std::size_t i = 0; i < clusters.size(); i++)
        {
            for (auto &customer : clusters[i])
            {
                clusters_mapping[customer] = i;
            }
        }

        // Filter out truckable customers (due to waiting time limit, for example)
        std::vector<bool> real_truckable(problem->customers.size());
        if (problem->trucks_count > 0)
        {
            real_truckable[0] = true;
            std::vector<std::vector<DroneRoute>> drone_routes(problem->drones_count);
            for (std::size_t i = 1; i < problem->customers.size(); i++)
            {
                std::vector<std::vector<d2d::TruckRoute>> truck_routes(problem->trucks_count);
                truck_routes[0].emplace_back(std::vector<std::size_t>{0, i, 0});
                if (_insertable<ST>(truck_routes, drone_routes))
                {
                    real_truckable[i] = true;
                }
            }
        }
        // std::cerr << "real_truckable = " << std::accumulate(real_truckable.begin(), real_truckable.end(), 0) << std::endl;

        // Filter out dronable customers who cannot form a single feasible route
        std::vector<bool> real_dronable(problem->customers.size());
        if (problem->drones_count > 0)
        {
            real_dronable[0] = true;
            std::vector<std::vector<d2d::TruckRoute>> truck_routes(problem->trucks_count);
            for (std::size_t i = 1; i < problem->customers.size(); i++)
            {
                if (problem->customers[i].dronable)
                {
                    std::vector<std::vector<DroneRoute>> drone_routes(problem->drones_count);
                    drone_routes[0].emplace_back(std::vector<std::size_t>{0, i, 0});
                    if (_insertable<ST>(truck_routes, drone_routes))
                    {
                        real_dronable[i] = true;
                    }
                }
            }
        }
        // std::cerr << "real_dronable = " << std::accumulate(real_dronable.begin(), real_dronable.end(), 0) << std::endl;

        const auto truckable = [&real_truckable](std::size_t c)
        {
            return real_truckable[c];
        };
        const auto dronable = [&real_dronable](std::size_t c)
        {
            return real_dronable[c];
        };

        // Ensure each customer can be served by at least 1 type of vehicle
        for (std::size_t i = 0; i < problem->customers.size(); i++)
        {
            if (!truckable(i) && !dronable(i))
            {
                throw std::runtime_error(utils::format("Customer %d cannot be served by neither trucks nor drones", i));
            }
        }

        std::multiset<_initialization_iteration_pack> timestamps;
        for (std::size_t i = 0; i < clusters.size(); i++)
        {
            // std::cerr << "At cluster " << i << ": " << clusters[i] << std::endl;
            if (clusters[i].empty())
            {
                continue;
            }

            std::sort(
                clusters[i].begin(), clusters[i].end(), [&problem](const std::size_t &i, const std::size_t &j)
                { return problem->distances[0][i] < problem->distances[0][j]; });
            // std::cerr << "Sorted to " << clusters[i] << std::endl;

            auto truckable_iter = std::find_if(clusters[i].begin(), clusters[i].end(), truckable);
            if (truckable_iter != clusters[i].end())
            {
                timestamps.emplace(0, i, 0, *truckable_iter, true);
            }

            auto dronable_iter = std::find_if(clusters[i].begin(), clusters[i].end(), dronable);
            if (dronable_iter != clusters[i].end())
            {
                timestamps.emplace(0, i, 0, *dronable_iter, false);
            }
        }

        std::set<std::size_t> global_customers;
        for (std::size_t i = 1; i < problem->customers.size(); i++)
        {
            global_customers.insert(global_customers.end(), i);
        }

        const auto truck_next = [&](std::size_t from, std::size_t truck) -> void
        {
            std::size_t nearest = std::numeric_limits<std::size_t>::max();
            double min_distance = std::numeric_limits<double>::max();
            for (auto &customer : clusters[clusters_mapping[from]])
            {
                if (truckable(customer) && problem->distances[from][customer] < min_distance)
                {
                    min_distance = problem->distances[from][customer];
                    nearest = customer;
                }
            }

            if (nearest == std::numeric_limits<std::size_t>::max()) // No truckable customers
            {
                for (auto &customer : global_customers)
                {
                    if (truckable(customer) && problem->distances[from][customer] < min_distance)
                    {
                        min_distance = problem->distances[from][customer];
                        nearest = customer;
                    }
                }
            }

            if (nearest < std::numeric_limits<std::size_t>::max())
            {
                auto temp = std::make_shared<ST>(truck_routes, drone_routes, nullptr, false);
                timestamps.emplace(temp->truck_working_time[truck], truck, from, nearest, true);
            }
        };

        const auto drone_next = [&](std::size_t from, std::size_t drone) -> void
        {
            std::size_t nearest = std::numeric_limits<std::size_t>::max();
            double min_distance = std::numeric_limits<double>::max();
            for (auto &customer : clusters[clusters_mapping[from]])
            {
                if (dronable(customer) && problem->distances[from][customer] < min_distance)
                {
                    min_distance = problem->distances[from][customer];
                    nearest = customer;
                }
            }

            if (nearest == std::numeric_limits<std::size_t>::max()) // No dronable customers
            {
                for (auto &customer : global_customers)
                {
                    if (dronable(customer) && problem->distances[from][customer] < min_distance)
                    {
                        min_distance = problem->distances[from][customer];
                        nearest = customer;
                    }
                }
            }

            if (nearest < std::numeric_limits<std::size_t>::max())
            {
                auto temp = std::make_shared<ST>(truck_routes, drone_routes, nullptr, false);
                timestamps.emplace(temp->drone_working_time[drone], drone, from, nearest, false);
            }
        };

        while (!global_customers.empty())
        {
            // std::cerr << "\e[31mtimestamps = " << timestamps << "\e[0m" << std::endl;
            // std::cerr << "global = " << global_customers << std::endl;
            const _initialization_iteration_pack packed(*timestamps.begin());
            timestamps.erase(timestamps.begin());

            std::size_t cluster = clusters_mapping[packed.customer];
            auto iter = std::find(clusters[cluster].begin(), clusters[cluster].end(), packed.customer);
            if (iter == clusters[cluster].end())
            {
                if (packed.is_truck)
                {
                    truck_next(packed.before, packed.vehicle);
                }
                else
                {
                    drone_next(packed.before, packed.vehicle);
                }

                continue;
            }

            clusters[cluster].erase(iter);
            global_customers.erase(packed.customer);

            if (packed.is_truck)
            {
                bool insertable;
                if (truck_routes[packed.vehicle].empty() || packed.before == 0)
                {
                    // std::cerr << "Constructing new route for truck " << packed.vehicle << " with customer " << packed.customer << std::endl;
                    insertable = _try_insert<ST>(truck_routes[packed.vehicle], packed.customer, truck_routes, drone_routes);
                }
                else
                {
                    // std::cerr << "Inserting to truck route " << truck_routes[packed.vehicle].back() << " with customer " << packed.customer << std::endl;
                    insertable = _try_insert<ST>(truck_routes[packed.vehicle].back(), packed.customer, truck_routes, drone_routes);
                }

                if (!insertable)
                {
                    // Re-insert
                    clusters[cluster].push_back(packed.customer);
                    global_customers.insert(packed.customer);

                    std::set<std::size_t> pool;
                    std::copy_if(
                        clusters[cluster].begin(), clusters[cluster].end(),
                        std::inserter(pool, pool.begin()),
                        truckable);
                    for (auto &p : timestamps)
                    {
                        pool.erase(p.customer);
                    }

                    // std::cerr << "pool = " << pool << ", timestamps = " << timestamps << std::endl;
                    if (pool.empty())
                    {
                        // std::cerr << "Utilizing global_customers" << std::endl;
                        std::copy_if(
                            global_customers.begin(), global_customers.end(),
                            std::inserter(pool, pool.begin()),
                            truckable);
                        for (auto &p : timestamps)
                        {
                            pool.erase(p.customer);
                        }
                    }

                    if (pool.empty())
                    {
                        // std::cerr << "\e[31mPool is empty!\e[0m" << std::endl;
                        continue;
                    }

                    auto next_customer = *std::max_element(
                        pool.begin(), pool.end(), [&problem](const std::size_t &i, const std::size_t &j)
                        { return problem->distances[0][i] < problem->distances[0][j]; });

                    // std::cerr << "next_customer = " << next_customer << std::endl;
                    timestamps.emplace(packed.working_time, packed.vehicle, 0, next_customer, true);
                }
                else
                {
                    // Insert to `timestamps`
                    truck_next(packed.customer, packed.vehicle);
                }
            }
            else
            {
                bool insertable;
                if (drone_routes[packed.vehicle].empty() || packed.before == 0)
                {
                    // std::cerr << "Constructing new route for drone " << packed.vehicle << " with customer " << packed.customer << std::endl;
                    insertable = _try_insert<ST>(drone_routes[packed.vehicle], packed.customer, truck_routes, drone_routes);
                }
                else
                {
                    // std::cerr << "Inserting to drone route " << drone_routes[packed.vehicle].back() << " with customer " << packed.customer << std::endl;
                    insertable = _try_insert<ST>(drone_routes[packed.vehicle].back(), packed.customer, truck_routes, drone_routes);
                }

                if (!insertable)
                {
                    // Re-insert
                    clusters[cluster].push_back(packed.customer);
                    global_customers.insert(packed.customer);

                    std::set<std::size_t> pool;
                    std::copy_if(
                        clusters[cluster].begin(), clusters[cluster].end(),
                        std::inserter(pool, pool.begin()),
                        dronable);
                    for (auto &p : timestamps)
                    {
                        pool.erase(p.customer);
                    }

                    // std::cerr << "pool = " << pool << ", timestamps = " << timestamps << std::endl;
                    if (pool.empty())
                    {
                        // std::cerr << "Utilizing global_customers" << std::endl;
                        std::copy_if(
                            global_customers.begin(), global_customers.end(),
                            std::inserter(pool, pool.begin()),
                            dronable);
                        for (auto &p : timestamps)
                        {
                            pool.erase(p.customer);
                        }
                    }

                    if (pool.empty())
                    {
                        // std::cerr << "\e[31mPool is empty!\e[0m" << std::endl;
                        continue;
                    }

                    auto next_customer = *std::min_element(
                        pool.begin(), pool.end(), [&problem](const std::size_t &i, const std::size_t &j)
                        { return problem->distances[0][i] < problem->distances[0][j]; });

                    timestamps.emplace(packed.working_time, packed.vehicle, 0, next_customer, false);
                }
                else
                {
                    // Insert to `timestamps`
                    drone_next(packed.customer, packed.vehicle);
                }
            }

            auto temp = std::make_shared<ST>(truck_routes, drone_routes, nullptr, false);
            // std::cerr << "truck_routes = " << truck_routes << " \e[31m" << temp->truck_working_time << "\e[0m" << std::endl;
            // std::cerr << "drone_routes = " << drone_routes << " \e[31m" << temp->drone_working_time << "\e[0m" << std::endl;
            // std::cerr << "feasible = " << temp->feasible << " " << temp->drone_energy_violation << " " << temp->capacity_violation << " " << temp->waiting_time_violation << " " << temp->fixed_time_violation << std::endl;
        }

        // Resize drone routes to `problem->drones_count`
        {
            std::vector<d2d::DroneRoute> all_routes;
            for (auto &routes : drone_routes)
            {
                for (auto &route : routes)
                {
                    all_routes.push_back(route);
                }
            }

            drone_routes.clear();
            drone_routes.resize(problem->drones_count);

            // Pair of maximum working time and difference between maximum and minimum working time among drones
            auto _optimal = std::make_pair(std::numeric_limits<double>::max(), 0.0);

            std::vector<std::pair<double, std::vector<d2d::DroneRoute *>>> _temp(problem->drones_count);
            std::vector<bool> _inserted(all_routes.size());

            auto _benchmark = std::make_unique<utils::PerformanceBenchmark>("Reorder drone routes");
            std::function<void(const std::size_t &, const std::size_t &)> _try;
            _try = [&](const std::size_t &drone, const std::size_t &inserted_count)
            {
                if (_benchmark->elapsed<std::chrono::seconds>().count() >= 10)
                {
                    return;
                }

                for (std::size_t i = 0; i < all_routes.size(); i++)
                {
                    if (!_inserted[i])
                    {
                        _inserted[i] = true;
                        _temp[drone].first += all_routes[i].working_time();
                        _temp[drone].second.push_back(&all_routes[i]);

                        if (_temp[drone].first < _optimal.first)
                        {
                            if (inserted_count + 1 == all_routes.size())
                            {
                                double maximum = 0, minimum = std::numeric_limits<double>::max();
                                for (auto &[working_time, _] : _temp)
                                {
                                    maximum = std::max(maximum, working_time);
                                    minimum = std::min(minimum, working_time);
                                }

                                auto cost = std::make_pair(maximum, maximum - minimum);

                                // Compensate for floating-point error
                                auto cost_cmp = std::make_pair(cost.first + 1e-4, cost.second + 1e-4);

                                if (cost_cmp < _optimal)
                                {
                                    _optimal = cost;
                                    // std::cerr << "_optimal = " << _optimal << std::endl;
                                    for (std::size_t drone = 0; drone < problem->drones_count; drone++)
                                    {
                                        drone_routes[drone].clear();
                                        for (auto &ptr : _temp[drone].second)
                                        {
                                            drone_routes[drone].push_back(*ptr);
                                        }
                                    }
                                }
                            }
                            else
                            {
                                for (std::size_t d = 0; d < problem->drones_count; d++)
                                {
                                    _try((drone + d + 1) % problem->drones_count, inserted_count + 1);
                                }
                            }
                        }

                        _temp[drone].second.pop_back();
                        _temp[drone].first -= all_routes[i].working_time();
                        _inserted[i] = false;
                    }
                }
            };

            _try(0, 0);
        }

        // TSP-optimizer
        std::vector<std::vector<TruckRoute>> truck_routes_modified;
        std::vector<std::vector<DroneRoute>> drone_routes_modified;

        for (auto &routes : truck_routes)
        {
            truck_routes_modified.emplace_back();
            for (auto &route : routes)
            {
                std::vector<std::size_t> customers(route.customers());
                customers.pop_back();
                customers.erase(customers.begin());

                _sort_cluster_with_starting_point(customers, 0);
                customers.insert(customers.begin(), 0);
                customers.push_back(0);

                truck_routes_modified.back().emplace_back(customers);
            }
        }
        for (auto &routes : drone_routes)
        {
            drone_routes_modified.emplace_back();
            for (auto &route : routes)
            {
                std::vector<std::size_t> customers(route.customers());
                customers.pop_back();
                customers.erase(customers.begin());

                _sort_cluster_with_starting_point(customers, 0);
                customers.insert(customers.begin(), 0);
                customers.push_back(0);

                drone_routes_modified.back().emplace_back(customers);
            }
        }

        return std::make_shared<ST>(
            truck_routes_modified,
            drone_routes_modified,
            std::make_shared<ParentInfo<ST>>(nullptr, utils::format("initial-%d", _Clusterizer)));
    }
}

namespace std
{
    ostream &operator<<(ostream &stream, const d2d::_initialization_iteration_pack &packed)
    {
        stream << "packed(working_time=" << packed.working_time;
        stream << ", vehicle=" << packed.vehicle;
        stream << ", before=" << packed.before;
        stream << ", customer=" << packed.customer;
        stream << ", is_truck=" << packed.is_truck << ")";
        return stream;
    }
}
