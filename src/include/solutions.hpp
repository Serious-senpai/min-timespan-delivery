#pragma once

#include "bitvector.hpp"
#include "tsp_solver.hpp"
#include "fp_specifier.hpp"
#include "initial.hpp"
#include "logger.hpp"
#include "parent.hpp"
#include "problem.hpp"
#include "routes.hpp"
#include "wrapper.hpp"
#include "neighborhoods/cross_3.hpp"
#include "neighborhoods/cross.hpp"
#include "neighborhoods/ejection_chain.hpp"
#include "neighborhoods/move_xy.hpp"
#include "neighborhoods/two_opt.hpp"

namespace d2d
{
    /** @brief Represents a solution to the D2D problem. */
    class Solution
    {
    private:
        static double A1, A2, A3, A4, B;

        static const std::vector<std::shared_ptr<Neighborhood<Solution, true>>> _neighborhoods;

        static std::vector<std::vector<std::vector<double>>> _calculate_truck_time_segments(
            const std::vector<std::vector<TruckRoute>> &truck_routes);
        static std::vector<double> _calculate_truck_working_time(
            const std::vector<std::vector<std::vector<double>>> &truck_time_segments);
        static std::vector<double> _calculate_drone_working_time(
            const std::vector<std::vector<DroneRoute>> &drone_routes);
        static double _calculate_working_time(
            const std::vector<double> &truck_working_time,
            const std::vector<double> &drone_working_time);
        static double _calculate_energy_violation(const std::vector<std::vector<DroneRoute>> &drone_routes);
        static double _calculate_capacity_violation(
            const std::vector<std::vector<TruckRoute>> &truck_routes,
            const std::vector<std::vector<DroneRoute>> &drone_routes);
        static double _calculate_waiting_time_violation(
            const std::vector<std::vector<TruckRoute>> &truck_routes,
            const std::vector<std::vector<std::vector<double>>> &truck_time_segments,
            const std::vector<std::vector<DroneRoute>> &drone_routes);
        static double _calculate_fixed_time_violation(const std::vector<std::vector<DroneRoute>> &drone_routes);

        const std::vector<std::vector<std::vector<double>>> _temp_truck_time_segments;

        const std::shared_ptr<ParentInfo<Solution>> _parent;

        template <typename RT, std::enable_if_t<is_route_v<RT>, bool> = true>
        void _hamming_distance(const std::vector<std::vector<RT>> &vehicle_routes, std::vector<std::size_t> &repr) const
        {
            auto problem = Problem::get_instance();
            repr.resize(problem->customers.size());

            for (auto &routes : vehicle_routes)
            {
                for (auto &route : routes)
                {
                    const std::vector<std::size_t> &customers = route.customers();
                    for (std::size_t i = 1; i + 2 < customers.size(); i++)
                    {
                        auto current = customers[i], next = customers[i + 1];
                        repr[current] = next;
                    }
                }
            }
        }

    public:
        static std::array<double, 4> penalty_coefficients();
        static std::vector<std::vector<double>> best_with_edges;

        /** @brief Working time of truck routes */
        const std::vector<double> truck_working_time;

        /** @brief Working time of drone routes */
        const std::vector<double> drone_working_time;

        /** @brief System working time */
        const double working_time;

        /** @brief Total drone energy violation */
        const double drone_energy_violation;

        /** @brief Total capacity violation */
        const double capacity_violation;

        /** @brief Total waiting time violation */
        const double waiting_time_violation;

        /** @brief Total fixed time violation */
        const double fixed_time_violation;

        /** @brief Routes of trucks */
        const std::vector<std::vector<TruckRoute>> truck_routes;

        /** @brief Routes of drones */
        const std::vector<std::vector<DroneRoute>> drone_routes;

        /** @brief Solution feasibility */
        const bool feasible;

        Solution(
            const std::vector<std::vector<TruckRoute>> &truck_routes,
            const std::vector<std::vector<DroneRoute>> &drone_routes,
            const std::shared_ptr<ParentInfo<Solution>> parent,
            const bool debug_check = true)
            : _temp_truck_time_segments(_calculate_truck_time_segments(truck_routes)),
              _parent(parent),
              truck_working_time(_calculate_truck_working_time(_temp_truck_time_segments)),
              drone_working_time(_calculate_drone_working_time(drone_routes)),
              working_time(_calculate_working_time(truck_working_time, drone_working_time)),
              drone_energy_violation(_calculate_energy_violation(drone_routes)),
              capacity_violation(_calculate_capacity_violation(truck_routes, drone_routes)),
              waiting_time_violation(_calculate_waiting_time_violation(truck_routes, _temp_truck_time_segments, drone_routes)),
              fixed_time_violation(_calculate_fixed_time_violation(drone_routes)),
              truck_routes(truck_routes),
              drone_routes(drone_routes),
              feasible(
                  utils::approximate(drone_energy_violation, 0.0) &&
                  utils::approximate(capacity_violation, 0.0) &&
                  utils::approximate(waiting_time_violation, 0.0) &&
                  utils::approximate(fixed_time_violation, 0.0))
        {
            const auto problem = Problem::get_instance();

            if (feasible && debug_check)
            {
                const auto n = problem->customers.size();
                best_with_edges.resize(n, std::vector<double>(n, std::numeric_limits<double>::max()));

                const auto _populate = [this]<typename RT, std::enable_if_t<is_route_v<RT>, bool> = true>(
                                           const std::vector<std::vector<RT>> &vehicle_routes)
                {
                    for (auto &routes : vehicle_routes)
                    {
                        for (auto &route : routes)
                        {
                            const auto &customers = route.customers();
                            for (std::size_t i = 0; i + 1 < customers.size(); i++)
                            {
                                double &current = best_with_edges[customers[i]][customers[i + 1]];
                                current = std::min(current, working_time);
                            }
                        }
                    }
                };
                _populate(truck_routes);
                _populate(drone_routes);
            }

            if (debug_check)
            {
#ifdef DEBUG
                if (truck_routes.size() != problem->trucks_count)
                {
                    throw std::runtime_error(utils::format("Expected %lu truck(s), not %lu", problem->trucks_count, truck_routes.size()));
                }
                if (drone_routes.size() != problem->drones_count)
                {
                    throw std::runtime_error(utils::format("Expected %lu drone(s), not %lu", problem->drones_count, drone_routes.size()));
                }

                std::vector<bool> exists(problem->customers.size());
                auto _constructor_check_exists = [&exists]<typename RT, std::enable_if_t<is_route_v<RT>, bool> = true>(const std::vector<std::vector<RT>> &vehicle_routes)
                {
                    for (auto &routes : vehicle_routes)
                    {
                        for (auto &route : routes)
                        {
                            for (auto &customer : route.customers())
                            {
                                if (exists[customer] && customer != 0)
                                {
                                    throw std::runtime_error(utils::format("Customer %lu is visited more than once", customer));
                                }

                                exists[customer] = true;
                            }
                        }
                    }
                };

                _constructor_check_exists(truck_routes);
                _constructor_check_exists(drone_routes);

                for (std::size_t i = 0; i < exists.size(); i++)
                {
                    if (!exists[i])
                    {
                        throw std::runtime_error(utils::format("Missing customer %lu", i));
                    }
                }

                if (parent == nullptr)
                {
                    throw std::runtime_error("Solution parent must not be a nullptr. Construct a parent with its `ptr` as a nullptr instead.");
                }
#endif
            }
        }

        /** @brief The parent solution propagating this solution in the result tree */
        std::shared_ptr<ParentInfo<Solution>> parent() const
        {
            return _parent;
        }

        std::shared_ptr<Solution> destroy_and_repair() const
        {
            const auto problem = Problem::get_instance();

            std::vector<std::vector<TruckRoute>> new_truck_routes(truck_routes);
            std::vector<std::vector<DroneRoute>> new_drone_routes(drone_routes);

            // Destroy phase
            std::set<std::size_t> move; // Set of destroyed customers

            auto destroy_count = problem->customers.size() * problem->destroy_rate / 100;
            while (move.size() < destroy_count)
            {
                // std::cerr << "move = " << move << std::endl;
                std::vector<std::size_t> scores(problem->customers.size());
                const auto _calculate_scores = [this, &scores]<typename RT, std::enable_if_t<is_route_v<RT>, bool> = true>(const std::vector<std::vector<RT>> &vehicle_routes)
                {
                    for (auto &routes : vehicle_routes)
                    {
                        for (auto &route : routes)
                        {
                            const auto &customers = route.customers();
                            for (std::size_t i = 1; i + 1 < customers.size(); i++)
                            {
                                scores[customers[i]] = best_with_edges[customers[i - 1]][customers[i]] + best_with_edges[customers[i]][customers[i + 1]];
                                // std::cerr << "scores[" << customers[i] << "] = " << best_with_edges[customers[i - 1]][customers[i]] << " + " << best_with_edges[customers[i]][customers[i + 1]] << " = " << scores[customers[i]] << std::endl;
                            }
                        }
                    }
                };

                _calculate_scores(new_truck_routes);
                _calculate_scores(new_drone_routes);
                // std::cerr << "scores = " << scores << std::endl;

                std::vector<std::size_t> customers;
                for (std::size_t i = 1; i < problem->customers.size(); i++)
                {
                    if (move.find(i) == move.end())
                    {
                        customers.push_back(i);
                    }
                }

                std::sort(
                    customers.begin(), customers.end(),
                    [&scores](const std::size_t &first, const std::size_t &second)
                    {
                        return scores[first] > scores[second];
                    });

                std::size_t index = static_cast<double>(customers.size()) * std::pow(utils::random<double>(0, 1), 2.0);

                const auto customer = customers[index];
                move.insert(customer);

                // std::cerr << "customers = " << customers << ", index = " << index << ", customer = " << customer << std::endl;

                const auto _destroy = [&customer]<typename RT, std::enable_if_t<is_route_v<RT>, bool> = true>(std::vector<std::vector<RT>> &vehicle_routes)
                {
                    for (std::size_t vehicle = 0; vehicle < vehicle_routes.size(); vehicle++)
                    {
                        for (std::size_t route = 0; route < vehicle_routes[vehicle].size(); route++)
                        {
                            std::vector<std::size_t> new_customers(vehicle_routes[vehicle][route].customers());
                            auto iter = std::find(new_customers.begin(), new_customers.end(), customer);
                            if (iter != new_customers.end())
                            {
                                new_customers.erase(iter);
                                if (new_customers.size() == 2)
                                {
                                    vehicle_routes[vehicle].erase(vehicle_routes[vehicle].begin() + route);
                                }
                                else
                                {
                                    vehicle_routes[vehicle][route] = RT(new_customers);
                                }
                                return true;
                            }
                        }
                    }

                    return false;
                };

                if (!_destroy(new_truck_routes))
                {
                    _destroy(new_drone_routes);
                }
            }

            // Repair phase
            std::vector<std::size_t> move_customers(move.begin(), move.end());
            std::shuffle(move_customers.begin(), move_customers.end(), utils::rng);

            const auto parent = std::make_shared<ParentInfo<Solution>>(std::make_shared<Solution>(*this), "destroy & repair");
            for (const auto &customer : move_customers)
            {
                double best_working_time = std::numeric_limits<double>::max();
                std::vector<std::vector<TruckRoute>> optimal_truck_routes;
                std::vector<std::vector<DroneRoute>> optimal_drone_routes;

                const auto _try_insert = [&]<typename RT, std::enable_if_t<is_route_v<RT>, bool> = true>(std::vector<std::vector<RT>> &vehicle_routes)
                {
                    for (auto &routes : vehicle_routes)
                    {
                        for (auto &route : routes)
                        {
                            const RT original(route);
                            const auto &customers = original.customers();
                            for (std::size_t i = 1; i + 1 < customers.size(); i++)
                            {
                                std::vector<std::size_t> new_customers(customers);
                                new_customers.insert(new_customers.begin() + i, customer);

                                // Temporary modify
                                route = RT(new_customers);

                                auto new_solution = std::make_shared<Solution>(new_truck_routes, new_drone_routes, parent, false);
                                if (new_solution->feasible && new_solution->working_time < best_working_time)
                                {
                                    best_working_time = new_solution->working_time;
                                    optimal_truck_routes = new_truck_routes;
                                    optimal_drone_routes = new_drone_routes;
                                }

                                // Restore
                                route = original;
                            }
                        }
                    }
                };

                _try_insert(new_truck_routes);
                if (problem->customers[customer].dronable)
                {
                    _try_insert(new_drone_routes);
                }

                if (best_working_time == std::numeric_limits<double>::max())
                {
                    // Insertion is not possible, we make new single-customer routes
                    const auto _try_append = [&]<typename RT, std::enable_if_t<is_route_v<RT>, bool> = true>(std::vector<std::vector<RT>> &vehicle_routes)
                    {
                        for (auto &routes : vehicle_routes)
                        {
                            routes.emplace_back(std::vector<std::size_t>{0, customer, 0});

                            auto new_solution = std::make_shared<Solution>(new_truck_routes, new_drone_routes, parent, false);
                            if (new_solution->feasible && new_solution->working_time < best_working_time)
                            {
                                best_working_time = new_solution->working_time;
                                optimal_truck_routes = new_truck_routes;
                                optimal_drone_routes = new_drone_routes;
                            }

                            // Restore
                            routes.pop_back();
                        }
                    };

                    _try_append(new_truck_routes);
                    if (problem->customers[customer].dronable)
                    {
                        _try_append(new_drone_routes);
                    }

                    if (best_working_time == std::numeric_limits<double>::max())
                    {
                        // std::cerr << new_truck_routes << std::endl;
                        // std::cerr << new_drone_routes << std::endl;

                        // This should never happen. Appending a new route to a feasible solution should always yield another feasible one.
                        throw std::runtime_error("Unreachable code was reached");
                    }
                    else
                    {
                        new_truck_routes = optimal_truck_routes;
                        new_drone_routes = optimal_drone_routes;
                    }
                }
                else
                {
                    new_truck_routes = optimal_truck_routes;
                    new_drone_routes = optimal_drone_routes;
                }
            }

            // std::cerr << "Destroy & repair from:\n";
            // std::cerr << truck_routes << " " << drone_routes << std::endl;
            // std::cerr << "Destroy & repair to:\n";
            // std::cerr << new_truck_routes << " " << new_drone_routes << std::endl;

            auto result = std::make_shared<Solution>(new_truck_routes, new_drone_routes, _parent);
            // std::cerr << "Hamming distance = \e[31m" << hamming_distance(result) << "\e[0m" << std::endl;
            // std::cerr << "Cost = " << cost() << " -> " << result->cost() << std::endl;
            return result;
        }

        /** @brief Objective function evaluation, including penalties. */
        utils::FloatingPointWrapper<double> cost() const
        {
            double result = working_time;
            result += A1 * drone_energy_violation;
            result += A2 * capacity_violation;
            result += A3 * waiting_time_violation;
            result += A4 * fixed_time_violation;

            return result;
        }

        double hamming_distance(const std::shared_ptr<Solution> other) const
        {
            std::vector<std::size_t> self_repr;
            _hamming_distance(truck_routes, self_repr);
            _hamming_distance(drone_routes, self_repr);

            std::vector<std::size_t> other_repr;
            _hamming_distance(other->truck_routes, other_repr);
            _hamming_distance(other->drone_routes, other_repr);

            std::size_t result = 0;
            for (std::size_t i = 0; i < self_repr.size(); i++)
            {
                if (self_repr[i] != other_repr[i])
                {
                    result++;
                }
            }

            return result;
        }

        std::shared_ptr<Solution> post_optimization(Logger<Solution> &logger)
        {
            auto problem = Problem::get_instance();
            std::size_t iteration = 0;

            std::vector<std::shared_ptr<BaseNeighborhood<Solution>>> inter_route, intra_route;
            for (auto &neighborhood : _neighborhoods)
            {
                inter_route.push_back(neighborhood);
                intra_route.push_back(neighborhood);
            }
            inter_route.push_back(std::make_shared<CrossExchange_3<Solution>>());
            inter_route.push_back(std::make_shared<CrossExchange<Solution>>());
            inter_route.push_back(std::make_shared<EjectionChain<Solution>>());

            auto result = std::make_shared<Solution>(*this);
            bool improved = true;
            auto aspiration_criteria = [&result, &improved](std::shared_ptr<Solution> s)
            {
                if (s->feasible && s->cost() < result->cost())
                {
                    result = s;
                    improved = true;
                }

                return true; // Accept all solutions in post-optimization
            };

            while (improved)
            {
                improved = false;
                std::shuffle(inter_route.begin(), inter_route.end(), utils::rng);
                for (auto &neighborhood : inter_route)
                {
                    if (problem->verbose)
                    {
                        std::cerr << utils::format("\rPost-optimize #%lu(%.2lf)", ++iteration, result->cost()) << std::flush;
                    }

                    auto ptr = std::dynamic_pointer_cast<Neighborhood<Solution, true>>(neighborhood);
                    if (ptr != nullptr)
                    {
                        ptr->clear();
                    }

                    neighborhood->inter_route(result, aspiration_criteria);

#ifdef LOGGING
                    logger.log(
                        result,
                        result,
                        {},
                        std::make_pair(
                            neighborhood->label() + "/post-optimization/inter-route",
                            ptr == nullptr ? std::vector<std::size_t>() : ptr->last_tabu()));
#endif
                }
            }

            improved = true;
            while (improved)
            {
                improved = false;
                std::shuffle(intra_route.begin(), intra_route.end(), utils::rng);
                for (auto &neighborhood : intra_route)
                {
                    if (problem->verbose)
                    {
                        std::cerr << utils::format("\rPost-optimize #%lu(%.2lf)", ++iteration, result->cost()) << std::flush;
                    }

                    auto ptr = std::dynamic_pointer_cast<Neighborhood<Solution, true>>(neighborhood);
                    if (ptr != nullptr)
                    {
                        ptr->clear();
                    }

                    neighborhood->intra_route(result, aspiration_criteria);

#ifdef LOGGING
                    logger.log(
                        result,
                        result,
                        {},
                        std::make_pair(
                            neighborhood->label() + "/post-optimization/intra-route",
                            ptr == nullptr ? std::vector<std::size_t>() : ptr->last_tabu()));
#endif
                }
            }

            std::vector<std::vector<TruckRoute>> new_truck_routes(result->truck_routes);
            std::vector<std::vector<DroneRoute>> new_drone_routes(result->drone_routes);

            auto parent = std::make_shared<ParentInfo<Solution>>(result, "TSP optimization");
            auto optimize_route = [&problem, &result, &new_truck_routes, &new_drone_routes, &parent]<typename RT, std::enable_if_t<is_route_v<RT>, bool> = true>(std::vector<std::vector<RT>> &vehicle_routes)
            {
                for (auto &routes : vehicle_routes)
                {
                    for (auto &route : routes)
                    {
                        RT old_route(route);

                        std::vector<std::size_t> customers(route.customers());
                        customers.pop_back();

                        auto distance = [&problem, &customers](const std::size_t &i, const std::size_t &j)
                        {
                            return problem->distances[customers[i]][customers[j]];
                        };

                        std::vector<std::size_t> ordered(customers.size());
                        std::iota(ordered.begin(), ordered.end(), 0);
                        ordered = customers.size() < 23 ? utils::held_karp_algorithm(customers.size(), distance).second
                                                        : utils::two_opt_heuristic(customers.size(), distance, ordered).second;

                        std::vector<std::size_t> new_customers(customers.size());
                        std::transform(
                            ordered.begin(), ordered.end(), new_customers.begin(),
                            [&customers](const std::size_t &i)
                            { return customers[i]; });

                        std::rotate(
                            new_customers.begin(),
                            std::find(new_customers.begin(), new_customers.end(), 0),
                            new_customers.end());

                        new_customers.push_back(0);
                        route = RT(new_customers);

                        auto new_solution = std::make_shared<Solution>(new_truck_routes, new_drone_routes, parent);
                        if (!new_solution->feasible || new_solution->cost() >= result->cost())
                        {
                            route = old_route;
                        }
                    }
                }
            };

            optimize_route(new_truck_routes);
            optimize_route(new_drone_routes);
            result = std::make_shared<Solution>(new_truck_routes, new_drone_routes, parent);

            if (problem->verbose)
            {
                std::cerr << std::endl;
            }

            return result;
        }

        bool operator==(const Solution &other) const
        {
            return truck_routes == other.truck_routes && drone_routes == other.drone_routes;
        }

        bool operator!=(const Solution &other) const
        {
            return !(*this == other);
        }

        static std::shared_ptr<Solution> tabu_search(Logger<Solution> &logger);
    };

    double Solution::A1 = 1;
    double Solution::A2 = 1;
    double Solution::A3 = 1;
    double Solution::A4 = 1;
    double Solution::B = 1.5;

    const std::vector<std::shared_ptr<Neighborhood<Solution, true>>> Solution::_neighborhoods = {
        std::make_shared<MoveXY<Solution, 1, 0>>(),
        std::make_shared<MoveXY<Solution, 1, 1>>(),
        std::make_shared<MoveXY<Solution, 2, 0>>(),
        std::make_shared<MoveXY<Solution, 2, 1>>(),
        std::make_shared<MoveXY<Solution, 2, 2>>(),
        std::make_shared<TwoOpt<Solution>>(),
    };

    std::vector<std::vector<std::vector<double>>> Solution::_calculate_truck_time_segments(
        const std::vector<std::vector<TruckRoute>> &truck_routes)
    {
        std::vector<std::vector<std::vector<double>>> result(truck_routes.size());
        for (std::size_t i = 0; i < truck_routes.size(); i++)
        {
            result[i].reserve(truck_routes[i].size());

            std::size_t coefficients_index = 0;
            double current_within_timespan = 0;
            for (std::size_t j = 0; j < truck_routes[i].size(); j++)
            {
                result[i].push_back(TruckRoute::calculate_time_segments(
                    truck_routes[i][j].customers(),
                    coefficients_index,
                    current_within_timespan));
            }
        }

        return result;
    }

    std::vector<double> Solution::_calculate_truck_working_time(const std::vector<std::vector<std::vector<double>>> &truck_time_segments)
    {
        std::vector<double> result;
        result.reserve(truck_time_segments.size());

        for (auto &routes : truck_time_segments)
        {
            double time = 0;
            for (auto &route : routes)
            {
                time += std::accumulate(route.begin(), route.end(), 0.0);
            }
            result.push_back(time);
        }

        return result;
    }

    std::vector<double> Solution::_calculate_drone_working_time(const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        std::vector<double> result;
        result.reserve(drone_routes.size());

        for (auto &routes : drone_routes)
        {
            double time = 0;
            for (auto &route : routes)
            {
                time += route.working_time();
            }
            result.push_back(time);
        }

        return result;
    }

    double Solution::_calculate_working_time(
        const std::vector<double> &truck_working_time,
        const std::vector<double> &drone_working_time)
    {
        double result = 0;
        if (!truck_working_time.empty())
        {
            result = *std::max_element(truck_working_time.begin(), truck_working_time.end());
        }
        if (!drone_working_time.empty())
        {
            result = std::max(result, *std::max_element(drone_working_time.begin(), drone_working_time.end()));
        }

        return result;
    }

    double Solution::_calculate_energy_violation(const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        double result = 0;
        auto problem = Problem::get_instance();
        if (problem->endurance == nullptr)
        {
            for (auto &routes : drone_routes)
            {
                for (auto &route : routes)
                {
                    result += route.energy_violation();
                }
            }
        }

        return result;
    }

    double Solution::_calculate_capacity_violation(
        const std::vector<std::vector<TruckRoute>> &truck_routes,
        const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        double result = 0;

        auto calculate = [&result]<typename RT, std::enable_if_t<is_route_v<RT>, bool> = true>(const std::vector<std::vector<RT>> &vehicle_routes)
        {
            for (auto &routes : vehicle_routes)
            {
                for (auto &route : routes)
                {
                    result += route.capacity_violation();
                }
            }
        };

        calculate(truck_routes);
        calculate(drone_routes);

        return result;
    }

    double Solution::_calculate_waiting_time_violation(
        const std::vector<std::vector<TruckRoute>> &truck_routes,
        const std::vector<std::vector<std::vector<double>>> &truck_time_segments,
        const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        double result = 0;

        for (std::size_t i = 0; i < truck_routes.size(); i++)
        {
            for (std::size_t j = 0; j < truck_routes[i].size(); j++)
            {
                auto waiting_time_violations = TruckRoute::calculate_waiting_time_violations(
                    truck_routes[i][j].customers(),
                    truck_time_segments[i][j]);
                result += std::accumulate(waiting_time_violations.begin(), waiting_time_violations.end(), 0.0);
            }
        }
        for (auto &routes : drone_routes)
        {
            for (auto &route : routes)
            {
                const std::vector<double> &waiting_time_violations = route.waiting_time_violations();
                result += std::accumulate(waiting_time_violations.begin(), waiting_time_violations.end(), 0.0);
            }
        }

        return result;
    }

    double Solution::_calculate_fixed_time_violation(const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        double result = 0;
        auto problem = Problem::get_instance();
        if (problem->endurance != nullptr)
        {
            for (auto &routes : drone_routes)
            {
                for (auto &route : routes)
                {
                    result += route.fixed_time_violation();
                }
            }
        }

        return result;
    }

    std::array<double, 4> Solution::penalty_coefficients()
    {
        return {A1, A2, A3, A4};
    }

    std::vector<std::vector<double>> Solution::best_with_edges;

    std::shared_ptr<Solution> Solution::tabu_search(Logger<Solution> &logger)
    {
        auto problem = Problem::get_instance();
        auto initial_1 = initial_impl<d2d::Solution, 1>(), initial_2 = initial_impl<d2d::Solution, 2>();

        std::vector<std::shared_ptr<Solution>> elite;
        if (initial_1->feasible)
        {
            elite.push_back(initial_1);
        }
        if (initial_2->feasible)
        {
            elite.push_back(initial_2);
        }

        auto current = initial_1->cost() < initial_2->cost() ? initial_1 : initial_2, result = current;

        std::size_t base_hyperparameter = (problem->customers.size() - 1) /
                                          (std::accumulate(
                                               current->truck_routes.begin(), current->truck_routes.end(), 0,
                                               [](const std::size_t &s, const std::vector<TruckRoute> &routes)
                                               { return s + !routes.empty(); }) +
                                           std::accumulate(
                                               current->drone_routes.begin(), current->drone_routes.end(), 0,
                                               [](const std::size_t &s, const std::vector<DroneRoute> &routes)
                                               { return s + !routes.empty(); }));

        problem->tabu_size = problem->tabu_size_factor * base_hyperparameter;
        problem->reset_after = problem->reset_after_factor * base_hyperparameter;

        std::cerr << "tabu_size = " << problem->tabu_size << "\n";
        std::cerr << "verbose = " << problem->verbose << "\n";
        std::cerr << "trucks_count = " << problem->trucks_count << ", drones_count = " << problem->drones_count << "\n";
        std::cerr << "strategy = " << problem->strategy << "\n";
        std::cerr << "waiting_time_limit = " << problem->waiting_time_limit << "\n";
        std::cerr << "max_elite_size = " << problem->max_elite_size << ", reset_after = " << problem->reset_after << "\n";

        logger.last_improved = 0;
        logger.iterations = 0;

        std::size_t neighborhood = 0;
        auto insert_elite = [&problem, &elite, &result]()
        {
            if (problem->max_elite_size == 0)
            {
                return;
            }

            if (elite.size() == problem->max_elite_size)
            {
                auto nearest = std::min_element(
                    elite.begin(), elite.end(),
                    [&result](const std::shared_ptr<Solution> &first, const std::shared_ptr<Solution> &second)
                    {
                        return result->hamming_distance(first) < result->hamming_distance(second);
                    });

                elite.erase(nearest);
            }

            elite.push_back(result);
        };

        std::size_t iteration_cap = (problem->fix_iteration > 0 ? problem->fix_iteration : std::numeric_limits<int>::max());
        for (std::size_t iteration = 0; iteration < iteration_cap; iteration++)
        {
            if (problem->verbose)
            {
                std::string format_string = utils::format(
                    "Iteration #%lu(%s/%s)",
                    iteration + 1,
                    utils::fp_format_specifier(current->cost()),
                    utils::fp_format_specifier(result->cost()));
                auto prefix = utils::format(format_string, current->cost(), result->cost());
                std::cerr << prefix;

                try
                {
                    std::size_t width = utils::get_console_size(false).first;
                    if (width > prefix.size())
                    {
                        std::cerr << std::string(width - prefix.size(), ' '); // Clear the remaining space
                    }
                }
                catch (std::runtime_error &)
                {
                    // ignore
                }

                std::cerr << '\r' << std::flush;
            }

            logger.iterations = iteration + 1;

            const auto aspiration_criteria = [&logger, &result, &insert_elite, &iteration](std::shared_ptr<Solution> ptr)
            {
                if (ptr->feasible && ptr->cost() < result->cost() && (!result->feasible || ptr->working_time < result->working_time))
                {
                    result = ptr;
                    logger.last_improved = iteration;
                    insert_elite();
                    return true;
                }

                return false;
            };

            auto neighbor = _neighborhoods[neighborhood]->move(current, aspiration_criteria); // result is updated by aspiration_criteria
            auto old_current = current;
            if (logger.last_improved == iteration)
            {
                current = result;
            }
            else if (neighbor != nullptr)
            {
                current = neighbor;
            }

            // Pop from elite set
            if (iteration != logger.last_improved && (iteration - logger.last_improved) % problem->reset_after == 0)
            {
                if (elite.empty())
                {
                    break;
                }

                auto iter = utils::random_element(elite);
                current = (*iter)->destroy_and_repair();
                elite.erase(iter);

                for (auto &neighborhood : _neighborhoods)
                {
                    neighborhood->clear();
                }
            }

#ifdef LOGGING
            logger.log(
                result,
                current,
                elite,
                std::make_pair(_neighborhoods[neighborhood]->label(), _neighborhoods[neighborhood]->last_tabu()));
#endif

            const auto violation_update = [](double &A, const double &violation)
            {
                if (violation > 0)
                {
                    A *= B;
                }
                else
                {
                    A /= B;
                }

                if (A < 1e-3 || A > 1e5)
                {
                    A = 1;
                }
            };

            violation_update(A1, current->drone_energy_violation);
            violation_update(A2, current->capacity_violation);
            violation_update(A3, current->waiting_time_violation);
            violation_update(A4, current->fixed_time_violation);

            if (problem->strategy == "random")
            {
                neighborhood = utils::random<std::size_t>(0, _neighborhoods.size() - 1);
            }
            else if (problem->strategy == "cyclic")
            {
                neighborhood = (neighborhood + 1) % _neighborhoods.size();
            }
            else if (problem->strategy == "vns")
            {
                static std::size_t last_last_improved = 0;
                if (last_last_improved != logger.last_improved)
                {
                    // Found a new best solution
                    neighborhood = 0;
                }
                else
                {
                    // No new best solution found
                    neighborhood = (neighborhood + 1) % _neighborhoods.size();
                }

                last_last_improved = logger.last_improved;
            }
            else
            {
                throw std::invalid_argument(utils::format("Unrecognized strategy \"%s\"", problem->strategy.c_str()));
            }
        }

        if (problem->verbose)
        {
            std::cerr << std::endl;
        }

        auto post_opt = result->post_optimization(logger);
        return post_opt;
    }
}
