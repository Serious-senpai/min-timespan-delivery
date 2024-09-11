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

        static const std::vector<std::shared_ptr<Neighborhood<Solution, true>>> neighborhoods;

        static std::vector<std::vector<std::vector<double>>> _calculate_truck_time_segments(
            const std::vector<std::vector<TruckRoute>> &truck_routes);
        static double _calculate_travel_cost(
            const std::vector<std::vector<TruckRoute>> &truck_routes,
            const std::vector<std::vector<DroneRoute>> &drone_routes);
        static std::vector<double> _calculate_truck_working_time(
            const std::vector<std::vector<std::vector<double>>> &truck_time_segments);
        static std::vector<double> _calculate_drone_working_time(
            const std::vector<std::vector<DroneRoute>> &drone_routes);
        static double _calculate_energy_violation(const std::vector<std::vector<DroneRoute>> &drone_routes);
        static double _calculate_capacity_violation(
            const std::vector<std::vector<TruckRoute>> &truck_routes,
            const std::vector<std::vector<DroneRoute>> &drone_routes);
        static double _calculate_working_time_violation(
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

        /** @brief Working time of truck routes */
        const std::vector<double> truck_working_time;

        /** @brief Working time of drone routes */
        const std::vector<double> drone_working_time;

        /** @brief Total travel cost */
        const double travel_cost;

        /** @brief Total drone energy violation */
        const double drone_energy_violation;

        /** @brief Total capacity violation */
        const double capacity_violation;

        /** @brief Total working time violation */
        const double working_time_violation;

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
              travel_cost(_calculate_travel_cost(truck_routes, drone_routes)),
              drone_energy_violation(_calculate_energy_violation(drone_routes)),
              capacity_violation(_calculate_capacity_violation(truck_routes, drone_routes)),
              working_time_violation(_calculate_working_time_violation(_temp_truck_time_segments, drone_routes)),
              fixed_time_violation(_calculate_fixed_time_violation(drone_routes)),
              truck_routes(truck_routes),
              drone_routes(drone_routes),
              feasible(
                  utils::approximate(drone_energy_violation, 0.0) &&
                  utils::approximate(capacity_violation, 0.0) &&
                  utils::approximate(working_time_violation, 0.0) &&
                  utils::approximate(fixed_time_violation, 0.0))
        {
            if (debug_check)
            {
#ifdef DEBUG
                auto problem = Problem::get_instance();
                if (truck_routes.size() != problem->trucks_count)
                {
                    throw std::runtime_error(utils::format("Expected %lu truck(s), not %lu", problem->trucks_count, truck_routes.size()));
                }
                if (drone_routes.size() != problem->drones_count)
                {
                    throw std::runtime_error(utils::format("Expected %lu drone(s), not %lu", problem->drones_count, drone_routes.size()));
                }

                for (auto &routes : truck_routes)
                {
                    if (routes.size() > 1)
                    {
                        throw std::runtime_error(utils::format("Number of truck routes must be at most 1, not %lu", routes.size()));
                    }
                }

                for (auto &routes : drone_routes)
                {
                    for (auto &route : routes)
                    {
                        if (route.customers().size() != 3)
                        {
                            throw std::runtime_error(utils::format("Each drone route can only served 1 customer, not %lu", route.customers().size() - 2));
                        }
                    }
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

#endif
            }
        }

        /** @brief The parent solution propagating this solution in the result tree */
        std::shared_ptr<ParentInfo<Solution>> parent() const
        {
            return _parent;
        }

        /** @brief Objective function evaluation, including penalties. */
        utils::FloatingPointWrapper<double> cost() const
        {
            double result = travel_cost;
            result += A1 * drone_energy_violation;
            result += A2 * capacity_violation;
            result += A3 * working_time_violation;
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
            for (auto &neighborhood : neighborhoods)
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

                    logger.log(
                        result,
                        result,
                        {},
                        std::make_pair(
                            neighborhood->label() + "/post-optimization/inter-route",
                            ptr == nullptr ? std::vector<std::size_t>() : ptr->last_tabu()));
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

                    logger.log(
                        result,
                        result,
                        {},
                        std::make_pair(
                            neighborhood->label() + "/post-optimization/intra-route",
                            ptr == nullptr ? std::vector<std::size_t>() : ptr->last_tabu()));
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
                            if constexpr (std::is_same_v<RT, TruckRoute>)
                            {
                                return problem->man_distances[customers[i]][customers[j]];
                            }
                            else
                            {
                                return problem->euc_distances[customers[i]][customers[j]];
                            }
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

    const std::vector<std::shared_ptr<Neighborhood<Solution, true>>> Solution::neighborhoods = {
        std::make_shared<MoveXY<Solution, 1, 0>>(),
        std::make_shared<MoveXY<Solution, 1, 1>>(),
        std::make_shared<MoveXY<Solution, 2, 0>>(),
        std::make_shared<MoveXY<Solution, 2, 1>>(),
        std::make_shared<MoveXY<Solution, 2, 2>>(),
        std::make_shared<TwoOpt<Solution>>()};

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

    double Solution::_calculate_travel_cost(
        const std::vector<std::vector<TruckRoute>> &truck_routes,
        const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        auto problem = Problem::get_instance();
        double result = 0;

        for (auto &routes : truck_routes)
        {
            for (auto &route : routes)
            {
                result += route.distance() * problem->truck_unit_cost;
            }
        }
        for (auto &routes : drone_routes)
        {
            for (auto &route : routes)
            {
                result += route.distance() * problem->drone_unit_cost;
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

    double Solution::_calculate_working_time_violation(
        const std::vector<std::vector<std::vector<double>>> &truck_time_segments,
        const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        auto problem = Problem::get_instance();
        double result = 0;

        for (auto &routes : truck_time_segments)
        {
            double time = 0;
            for (auto &route : routes)
            {
                time += std::accumulate(route.begin(), route.end(), 0.0);
            }
            result += std::max(0.0, time - problem->truck_time_limit);
        }
        for (auto &routes : drone_routes)
        {
            double time = 0;
            for (auto &route : routes)
            {
                time += route.working_time();
            }
            result += std::max(0.0, time - problem->drone_time_limit);
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

    std::shared_ptr<Solution> Solution::tabu_search(Logger<Solution> &logger)
    {
        auto problem = Problem::get_instance();
        std::vector<std::shared_ptr<Solution>> elite = {initial_impl<d2d::Solution, 1>(), initial_impl<d2d::Solution, 2>()};
        auto current = elite[0]->cost() < elite[1]->cost() ? elite[0] : elite[1], result = current;

        logger.last_improved = 0;
        logger.iterations = 0;

        std::size_t neighborhood = 0;
        auto insert_elite = [&problem, &elite, &result]()
        {
            std::sort(
                elite.begin(), elite.end(),
                [&result](const std::shared_ptr<Solution> &first, const std::shared_ptr<Solution> &second)
                {
                    return result->hamming_distance(first) < result->hamming_distance(second);
                });

            if (elite.size() == 10 || (!elite.empty() && result->hamming_distance(elite.front()) <= problem->hamming_distance_factor * problem->customers.size()))
            {
                elite.erase(elite.begin());
            }
            elite.push_back(result);
        };

        for (std::size_t iteration = 0;; iteration++)
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
                if (ptr->feasible && ptr->cost() < result->cost())
                {
                    result = ptr;
                    logger.last_improved = iteration;
                    insert_elite();
                    return true;
                }

                return false;
            };

            auto neighbor = neighborhoods[neighborhood]->move(current, aspiration_criteria); // result is updated by aspiration_criteria
            auto current_cost = current->cost();
            if (logger.last_improved == iteration)
            {
                current = result;
            }
            else if (neighbor != nullptr)
            {
                current = neighbor;
            }

            if (iteration != logger.last_improved && (iteration - logger.last_improved) % problem->reset_after == 0)
            {
                if (elite.empty())
                {
                    break;
                }

                auto iter = utils::random_element(elite);
                current = *iter;
                elite.erase(iter);
            }

            logger.log(
                result,
                current,
                elite,
                std::make_pair(neighborhoods[neighborhood]->label(), neighborhoods[neighborhood]->last_tabu()));

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
            violation_update(A3, current->working_time_violation);
            violation_update(A4, current->fixed_time_violation);

            /* Compare current_cost with current->cost() AFTER updating penalty coefficients */
            if (neighbor == nullptr || current_cost <= current->cost())
            {
                neighborhood = (neighborhood + 1) % neighborhoods.size();
            }
            else
            {
                neighborhood = 0;
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
