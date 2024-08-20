#pragma once

#include "bitvector.hpp"
#include "held_karp.hpp"
#include "initial.hpp"
#include "problem.hpp"
#include "routes.hpp"
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
        static double A1, A2, A3, A4, A5, B;

        static const std::vector<std::shared_ptr<Neighborhood<Solution, true>>> neighborhoods;

        static std::vector<std::vector<std::vector<double>>> _calculate_truck_time_segments(
            const std::vector<std::vector<TruckRoute>> &truck_routes);
        static double _calculate_working_time(
            const std::vector<std::vector<std::vector<double>>> &truck_time_segments,
            const std::vector<std::vector<DroneRoute>> &drone_routes);
        static double _calculate_energy_violation(const std::vector<std::vector<DroneRoute>> &drone_routes);
        static double _calculate_capacity_violation(
            const std::vector<std::vector<TruckRoute>> &truck_routes,
            const std::vector<std::vector<DroneRoute>> &drone_routes);
        static double _calculate_waiting_time_violation(
            const std::vector<std::vector<TruckRoute>> &truck_routes,
            const std::vector<std::vector<std::vector<double>>> &truck_time_segments,
            const std::vector<std::vector<DroneRoute>> &drone_routes);
        static double _calculate_fixed_time_violation(const std::vector<std::vector<DroneRoute>> &drone_routes);
        static double _calculate_fixed_distance_violation(const std::vector<std::vector<DroneRoute>> &drone_routes);

        const std::vector<std::vector<std::vector<double>>> _temp_truck_time_segments;

        std::shared_ptr<Solution> _parent;

    public:
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

        /** @brief Total fixed distance violation */
        const double fixed_distance_violation;

        /** @brief Routes of trucks */
        const std::vector<std::vector<TruckRoute>> truck_routes;

        /** @brief Routes of drones */
        const std::vector<std::vector<DroneRoute>> drone_routes;

        /** @brief Solution feasibility */
        const bool feasible;

        Solution(
            const std::vector<std::vector<TruckRoute>> &truck_routes,
            const std::vector<std::vector<DroneRoute>> &drone_routes)
            : _temp_truck_time_segments(_calculate_truck_time_segments(truck_routes)),
              working_time(_calculate_working_time(_temp_truck_time_segments, drone_routes)),
              drone_energy_violation(_calculate_energy_violation(drone_routes)),
              capacity_violation(_calculate_capacity_violation(truck_routes, drone_routes)),
              waiting_time_violation(_calculate_waiting_time_violation(truck_routes, _temp_truck_time_segments, drone_routes)),
              fixed_time_violation(_calculate_fixed_time_violation(drone_routes)),
              fixed_distance_violation(_calculate_fixed_distance_violation(drone_routes)),
              truck_routes(truck_routes),
              drone_routes(drone_routes),
              feasible(
                  utils::approximate(drone_energy_violation, 0.0) &&
                  utils::approximate(capacity_violation, 0.0) &&
                  utils::approximate(waiting_time_violation, 0.0) &&
                  utils::approximate(fixed_time_violation, 0.0) &&
                  utils::approximate(fixed_distance_violation, 0.0))
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

            std::vector<bool> exists(problem->customers.size());

#define CHECK_ROUTES(vehicle_routes)                                                                             \
    for (auto &routes : vehicle_routes)                                                                          \
    {                                                                                                            \
        for (auto &route : routes)                                                                               \
        {                                                                                                        \
            for (auto &customer : route.customers())                                                             \
            {                                                                                                    \
                if (exists[customer] && customer != 0)                                                           \
                {                                                                                                \
                    throw std::runtime_error(utils::format("Customer %lu is visited more than once", customer)); \
                }                                                                                                \
                                                                                                                 \
                exists[customer] = true;                                                                         \
            }                                                                                                    \
        }                                                                                                        \
    }

            CHECK_ROUTES(truck_routes);
            CHECK_ROUTES(drone_routes);
#undef CHECK_ROUTES

            for (std::size_t i = 0; i < exists.size(); i++)
            {
                if (!exists[i])
                {
                    throw std::runtime_error(utils::format("Missing customer %lu", i));
                }
            }

#endif
        }

        /** @brief The parent solution propagating this solution in the result tree */
        std::shared_ptr<Solution> parent() const
        {
            return _parent;
        }

        /** @brief Objective function evaluation, including penalties. */
        double cost() const
        {
            double result = working_time;
            result += A1 * drone_energy_violation;
            result += A2 * capacity_violation;
            result += A3 * waiting_time_violation;
            result += A4 * fixed_time_violation;
            result += A5 * fixed_distance_violation;

            return result;
        }

        double hamming_distance(std::shared_ptr<Solution> other) const
        {
            auto problem = Problem::get_instance();
            const std::size_t n = problem->customers.size() - 1;

#define OPERATION(vehicle_routes, repr)                                        \
    {                                                                          \
        for (auto &routes : vehicle_routes)                                    \
        {                                                                      \
            for (auto &route : routes)                                         \
            {                                                                  \
                const std::vector<std::size_t> &customers = route.customers(); \
                for (std::size_t i = 1; i + 2 < customers.size(); i++)         \
                {                                                              \
                    auto current = customers[i], next = customers[i + 1];      \
                    repr[current] = next;                                      \
                }                                                              \
            }                                                                  \
        }                                                                      \
    }

            std::vector<std::size_t> self_repr(n);
            OPERATION(truck_routes, self_repr);
            OPERATION(drone_routes, self_repr);

            std::vector<std::size_t> other_repr(n);
            OPERATION(other->truck_routes, other_repr);
            OPERATION(other->drone_routes, other_repr);

#undef OPERATION

            std::size_t result = 0;
            for (std::size_t i = 0; i < n; i++)
            {
                if (self_repr[i] != other_repr[i])
                {
                    result++;
                }
            }

            return result;
        }

        std::shared_ptr<Solution> post_optimization()
        {
            auto problem = Problem::get_instance();
            std::size_t iteration = 0;

            std::vector<std::shared_ptr<BaseNeighborhood<Solution>>> inter_route, intra_route;
            for (auto &neighborhood : neighborhoods)
            {
                inter_route.push_back(neighborhood);
                intra_route.push_back(neighborhood);
            }
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

                    neighborhood->multi_route(result, aspiration_criteria);
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

                    neighborhood->same_route(result, aspiration_criteria);
                }
            }

            if (problem->verbose)
            {
                std::cerr << std::endl;
            }

            return result;
        }

        static std::shared_ptr<Solution> initial();
        static std::shared_ptr<Solution> tabu_search(std::size_t *last_improved_ptr);
    };

    double Solution::A1 = 1;
    double Solution::A2 = 1;
    double Solution::A3 = 1;
    double Solution::A4 = 1;
    double Solution::A5 = 1;
    double Solution::B = 1.5;

    const std::vector<std::shared_ptr<Neighborhood<Solution, true>>> Solution::neighborhoods = {
        std::make_shared<MoveXY<Solution, 1, 0>>(),
        std::make_shared<MoveXY<Solution, 1, 1>>(),
        std::make_shared<MoveXY<Solution, 2, 0>>(),
        std::make_shared<MoveXY<Solution, 2, 1>>(),
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

    double Solution::_calculate_working_time(
        const std::vector<std::vector<std::vector<double>>> &truck_time_segments,
        const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        double result = 0;

        for (auto &routes : truck_time_segments)
        {
            double time = 0;
            for (auto &route : routes)
            {
                time += std::accumulate(route.begin(), route.end(), 0.0);
            }
            result = std::max(result, time);
        }
        for (auto &routes : drone_routes)
        {
            double time = 0;
            for (auto &route : routes)
            {
                time += route.working_time();
            }
            result = std::max(result, time);
        }

        return result;
    }

    double Solution::_calculate_energy_violation(const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        double result = 0;
        auto problem = Problem::get_instance();
        if (problem->endurance == nullptr)
        {
            const double battery = (problem->linear != nullptr) ? problem->linear->battery : problem->nonlinear->battery;
            for (auto &routes : drone_routes)
            {
                for (auto &route : routes)
                {
                    result += route.energy_violation() / battery;
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
        auto problem = Problem::get_instance();

#define CALCULATE_D2D_ROUTES(vehicle_routes)                                                \
    for (auto &routes : vehicle_routes)                                                     \
    {                                                                                       \
        for (auto &route : routes)                                                          \
        {                                                                                   \
            double capacity;                                                                \
            if constexpr (std::is_same_v<std::remove_cvref_t<decltype(route)>, TruckRoute>) \
            {                                                                               \
                capacity = problem->truck->capacity;                                        \
            }                                                                               \
            else                                                                            \
            {                                                                               \
                capacity = problem->drone->capacity;                                        \
            }                                                                               \
                                                                                            \
            result += route.capacity_violation() / capacity;                                \
        }                                                                                   \
    }

        CALCULATE_D2D_ROUTES(truck_routes);
        CALCULATE_D2D_ROUTES(drone_routes);
#undef CALCULATE_D2D_ROUTES

        return result;
    }

    double Solution::_calculate_waiting_time_violation(
        const std::vector<std::vector<TruckRoute>> &truck_routes,
        const std::vector<std::vector<std::vector<double>>> &truck_time_segments,
        const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        double result = 0;
        auto problem = Problem::get_instance();

        for (std::size_t i = 0; i < truck_routes.size(); i++)
        {
            for (std::size_t j = 0; j < truck_routes[i].size(); j++)
            {
                auto waiting_time_violations = TruckRoute::calculate_waiting_time_violations(
                    truck_routes[i][j].customers(),
                    truck_time_segments[i][j]);
                result += std::accumulate(
                    waiting_time_violations.begin(), waiting_time_violations.end(),
                    0.0,
                    [&problem](const double &result, const double &violation)
                    {
                        return result + violation / problem->maximum_waiting_time;
                    });
            }
        }
        for (auto &routes : drone_routes)
        {
            for (auto &route : routes)
            {
                const std::vector<double> &waiting_time_violations = route.waiting_time_violations();
                result += std::accumulate(
                    waiting_time_violations.begin(), waiting_time_violations.end(),
                    0.0,
                    [&problem](const double &result, const double &violation)
                    {
                        return result + violation / problem->maximum_waiting_time;
                    });
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
                    result += route.fixed_time_violation() / problem->endurance->fixed_time;
                }
            }
        }

        return result;
    }

    double Solution::_calculate_fixed_distance_violation(const std::vector<std::vector<DroneRoute>> &drone_routes)
    {
        double result = 0;
        auto problem = Problem::get_instance();
        if (problem->endurance != nullptr)
        {
            for (auto &routes : drone_routes)
            {
                for (auto &route : routes)
                {
                    result += route.fixed_distance_violation() / problem->endurance->fixed_distance;
                }
            }
        }

        return result;
    }

    std::shared_ptr<Solution> Solution::initial()
    {
        auto result = initial_12<Solution, true>();
        auto r = initial_12<Solution, false>();
        result = result->cost() < r->cost() ? result : r;

        r = initial_3<Solution>();
        result = result->cost() < r->cost() ? result : r;

        r = initial_4<Solution>();
        result = result->cost() < r->cost() ? result : r;

        return result;
    }

    std::shared_ptr<Solution> Solution::tabu_search(std::size_t *last_improved_ptr)
    {
        auto problem = Problem::get_instance();
        auto current = initial(), result = current;

        if (last_improved_ptr != nullptr)
        {
            *last_improved_ptr = 0;
        }

        std::size_t neighborhood = 0;
        for (std::size_t iteration = 0; iteration < problem->iterations; iteration++)
        {
            if (problem->verbose)
            {
                std::string format_string = "\rIteration #%lu/%lu(";
                format_string += current->cost() > 999999 ? "%.2e" : "%.2lf";
                format_string += "/";
                format_string += result->cost() > 999999 ? "%.2e" : "%.2lf";
                format_string += ") ";

                auto prefix = utils::format(format_string, iteration + 1, problem->iterations, current->cost(), result->cost());
                std::cerr << prefix;
                try
                {
                    auto width = utils::get_console_size(false).first;
                    const std::size_t excess = 10;
                    if (prefix.size() + excess < width)
                    {
                        auto total = width - prefix.size() - excess,
                             cover = (iteration * total + problem->iterations - 1) / problem->iterations;
                        std::cerr << '[' << std::string(cover, '#') << std::string(total - cover, ' ') << ']';
                    }
                }
                catch (std::runtime_error &)
                {
                    // pass
                }
                std::cerr << std::flush;
            }

            const auto aspiration_criteria = [&last_improved_ptr, &result, &iteration](std::shared_ptr<Solution> ptr)
            {
                if (ptr->feasible && ptr->cost() < result->cost())
                {
                    ptr->_parent = result;
                    result = ptr;
                    if (last_improved_ptr != nullptr)
                    {
                        *last_improved_ptr = iteration;
                    }

                    return true;
                }

                return false;
            };

            auto neighbor = neighborhoods[neighborhood]->move(current, aspiration_criteria);
            auto current_cost = current->cost();
            if (neighbor != nullptr)
            {
                current = neighbor;
                if (neighbor->feasible && neighbor->cost() < result->cost())
                {
                    neighbor->_parent = result;
                    result = neighbor;
                    if (last_improved_ptr != nullptr)
                    {
                        *last_improved_ptr = iteration;
                    }
                }
            }

            if (neighbor == nullptr || current_cost <= current->cost())
            {
                neighborhood = (neighborhood + 1) % neighborhoods.size();
            }
            else
            {
                neighborhood = 0;
            }

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
            };

            violation_update(A1, current->drone_energy_violation);
            violation_update(A2, current->capacity_violation);
            violation_update(A3, current->waiting_time_violation);
            violation_update(A4, current->fixed_time_violation);
            violation_update(A5, current->fixed_distance_violation);
        }

        if (problem->verbose)
        {
            std::cerr << std::endl;
        }

        auto post_opt = result->post_optimization();
        post_opt->_parent = result;
        return post_opt;
    }
}
