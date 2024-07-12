#pragma once

#include "random.hpp"
#include "routes.hpp"

namespace d2d
{
    class Solution; // forward declaration

    std::shared_ptr<Solution> initial_1()
    {
        auto problem = Problem::get_instance();
        std::vector<std::vector<TruckRoute>> truck_routes(problem->trucks_count);
        std::vector<std::vector<DroneRoute>> drone_routes(problem->trucks_count);

        std::vector<std::size_t> second_phase;
        // Begin first phase
        {
            std::vector<std::size_t> first_phase(problem->customers.size() - 1);
            std::iota(first_phase.begin(), first_phase.end(), 1);
            std::sort(
                first_phase.begin(), first_phase.end(),
                [&problem](const std::size_t &first, const std::size_t &second)
                {
                    return problem->distances[0][first] < problem->distances[0][second];
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
        const auto truck_insert = [](TruckRoute &route, const std::size_t &customer) -> bool
        {
            TruckRoute old(route);
            route.push_back(customer);
            if (route.waiting_time_violations().sum() > 0 || route.capacity_violation() > 0)
            {
                route = old;
                return false;
            }

            return true;
        };

        const auto drone_insert = [](DroneRoute &route, const std::size_t &customer) -> bool
        {
            DroneRoute old(route);
            route.push_back(customer);
            if (route.waiting_time_violations().sum() > 0 || route.capacity_violation() > 0 || route.energy_violation() > 0)
            {
                route = old;
                return false;
            }

            return true;
        };

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
                            if (drone_insert(route, customer))
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
                        if (truck_insert(route, customer))
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
        {
            std::size_t truck = 0, drone = 0;

            std::shuffle(third_phase.begin(), third_phase.end(), utils::rng);
            while (!third_phase.empty())
            {
                auto customer = third_phase.back();
                third_phase.pop_back();

                if (problem->customers[customer].dronable)
                {
                    if (!drone_insert(drone_routes[drone % problem->drones_count].back(), customer))
                    {
                        drone_routes[drone % problem->drones_count].push_back(DroneRoute({0, customer, 0}));
                    }

                    drone++;
                }
                else
                {
                    if (!truck_insert(truck_routes[truck % problem->trucks_count].back(), customer))
                    {
                        truck_routes[truck % problem->trucks_count].push_back(TruckRoute({0, customer, 0}));
                    }

                    truck++;
                }
            }
        }
        // End third phase

        return std::make_shared<Solution>(truck_routes, drone_routes);
    }
}
