#pragma once

#include "random.hpp"
#include "routes.hpp"

namespace d2d
{
    bool _truck_try_insert(TruckRoute &route, const std::size_t &customer)
    {
        TruckRoute old(route);
        route.push_back(customer);

        if (route.capacity_violation() > 0)
        {
            route = old;
            return false;
        }

        return true;
    };

    bool _drone_try_insert(DroneRoute &route, const std::size_t &customer)
    {
        DroneRoute old(route);
        route.push_back(customer);

        const std::vector<double> &wtv = route.waiting_time_violations();
        if (std::accumulate(wtv.begin(), wtv.end(), 0.0) > 0 || route.capacity_violation() > 0 || route.energy_violation() > 0)
        {
            route = old;
            return false;
        }

        return true;
    };

#define INITIAL_12_PHASE_3(problem, third_phase, truck_routes, drone_routes)                             \
    {                                                                                                    \
        std::size_t truck = 0, drone = 0;                                                                \
                                                                                                         \
        std::shuffle(third_phase.begin(), third_phase.end(), utils::rng);                                \
        while (!third_phase.empty())                                                                     \
        {                                                                                                \
            auto customer = third_phase.back();                                                          \
            third_phase.pop_back();                                                                      \
                                                                                                         \
            if (problem->customers[customer].dronable)                                                   \
            {                                                                                            \
                if (!_drone_try_insert(drone_routes[drone % problem->drones_count].back(), customer))    \
                {                                                                                        \
                    drone_routes[drone % problem->drones_count].push_back(DroneRoute({0, customer, 0})); \
                }                                                                                        \
                                                                                                         \
                drone++;                                                                                 \
            }                                                                                            \
            else                                                                                         \
            {                                                                                            \
                if (!_truck_try_insert(truck_routes[truck % problem->trucks_count].back(), customer))    \
                {                                                                                        \
                    truck_routes[truck % problem->trucks_count].push_back(TruckRoute({0, customer, 0})); \
                }                                                                                        \
                                                                                                         \
                truck++;                                                                                 \
            }                                                                                            \
        }                                                                                                \
    }

    template <typename ST, bool Nearest>
    std::shared_ptr<ST> initial_12()
    {
        auto problem = Problem::get_instance();
        std::vector<std::vector<TruckRoute>> truck_routes(problem->trucks_count);
        std::vector<std::vector<DroneRoute>> drone_routes(problem->drones_count);

        std::vector<std::size_t> second_phase;
        // Begin first phase
        {
            std::vector<std::size_t> first_phase(problem->customers.size() - 1);
            std::iota(first_phase.begin(), first_phase.end(), 1);
            std::sort(
                first_phase.begin(), first_phase.end(),
                [&problem](const std::size_t &first, const std::size_t &second)
                {
                    bool nearer = problem->euc_distances[0][first] < problem->euc_distances[0][second];
                    return Nearest ? nearer : !nearer;
                });

            auto truck_iter = truck_routes.begin();
            auto drone_iter = drone_routes.begin();
            for (auto &customer : first_phase)
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
                    second_phase.push_back(customer);
                }
            }
        }
        // End first phase

        std::vector<std::size_t> third_phase;
        // Begin second phase
        {
            for (auto &customer : second_phase)
            {
                if (problem->customers[customer].dronable)
                {
                    bool inserted = false;
                    for (auto &routes : drone_routes)
                    {
                        for (auto &route : routes)
                        {
                            if (_drone_try_insert(route, customer))
                            {
                                inserted = true;
                                break;
                            }
                        }

                        if (inserted)
                        {
                            break;
                        }
                    }

                    if (inserted)
                    {
                        continue;
                    }
                }

                bool inserted = false;
                for (auto &routes : truck_routes)
                {
                    for (auto &route : routes)
                    {
                        if (_truck_try_insert(route, customer))
                        {
                            inserted = true;
                            break;
                        }
                    }

                    if (inserted)
                    {
                        break;
                    }
                }

                if (!inserted)
                {
                    third_phase.push_back(customer);
                }
            }
        }
        // End second phase

        // Begin third phase
        INITIAL_12_PHASE_3(problem, third_phase, truck_routes, drone_routes);
        // End third phase

        return std::make_shared<ST>(truck_routes, drone_routes);
    }

    template <typename ST>
    std::shared_ptr<ST> initial_3()
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

        INITIAL_12_PHASE_3(problem, next_phase, truck_routes, drone_routes);

        return std::make_shared<ST>(truck_routes, drone_routes);
    }

#undef INITIAL_12_PHASE_3
}
