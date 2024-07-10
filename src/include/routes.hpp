#pragma once

#include "fenwick.hpp"
#include "problem.hpp"

namespace d2d
{
    class _BaseRoute
    {
    protected:
        static double _calculate_distance(const std::vector<std::size_t> &customers);
        static double _calculate_weight(const std::vector<std::size_t> &customers);
        static std::vector<double> _calculate_waiting_time_violations(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments,
            const std::function<double(const std::size_t &)> service_time);

    public:
        /**
         * @brief The order of customers in this route, starting and ending at the depot `0`.
         */
        const std::vector<std::size_t> customers;

        /**
         * @brief The time segments between each pair of adjacent customers.
         *
         * Given a pair of adjacent customers `(x, y)` in this route, a time segment is the time
         * from the moment the vehicle starts serving customer `x` to the moment it starts serving
         * customer `y` (including service time of `x` but not `y`).
         */
        const utils::FenwickTree<double> time_segments;

        /**
         * @brief The waiting time violations of each customer in this route.
         */
        const std::vector<double> waiting_time_violations;

        /**
         * @brief The total traveling distance of this route.
         */
        const double distance;

        /**
         * @brief The total customer demands of this route.
         */
        const double weight;

        /**
         * @brief The total working time of this route
         */
        const double working_time;

        _BaseRoute(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments,
            const std::vector<double> &waiting_time_violations,
            const double &distance,
            const double &weight)
            : customers(customers),
              time_segments(time_segments),
              waiting_time_violations(waiting_time_violations),
              distance(distance),
              weight(weight),
              working_time(time_segments.sum())
        {
#ifdef DEBUG
            if (customers.empty())
            {
                throw std::runtime_error("Routes must not be empty");
            }

            if (customers.front() != 0 || customers.back() != 0)
            {
                throw std::runtime_error("Routes must start and end at the depot");
            }
#endif
        }

        /** @brief The amount of weight exceeding vehicle capacity */
        virtual double weight_violation() const = 0;
    };

    double _BaseRoute::_calculate_distance(const std::vector<std::size_t> &customers)
    {
        auto problem = Problem::get_instance();
        double distance = 0;
        for (std::size_t i = 1; i < customers.size(); i++)
        {
            distance += problem->distances[customers[i - 1]][customers[i]];
        }

        return distance;
    }

    double _BaseRoute::_calculate_weight(const std::vector<std::size_t> &customers)
    {
        auto problem = Problem::get_instance();
        double weight = 0;
        for (auto &customer : customers)
        {
            weight += problem->customers[customer].demand;
        }

        return weight;
    }

    std::vector<double> _BaseRoute::_calculate_waiting_time_violations(
        const std::vector<std::size_t> &customers,
        const utils::FenwickTree<double> &time_segments,
        const std::function<double(const std::size_t &)> service_time)
    {
        auto problem = Problem::get_instance();
        std::vector<double> violations(customers.size());

        double time = time_segments.sum();
        for (std::size_t i = 0; i < customers.size(); i++)
        {
            violations[i] = std::max(0.0, time - service_time(customers[i]) - problem->maximum_waiting_time);
            time -= time_segments.get(i);
        }

        return violations;
    }

    /** @brief Represents a truck route. */
    class TruckRoute : public _BaseRoute
    {
    private:
        static utils::FenwickTree<double> _calculate_time_segments(const std::vector<std::size_t> &customers);
        static std::vector<double> _calculate_waiting_time_violations(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments);

    public:
        /** @brief Construct a `TruckRoute` with pre-calculated attributes */
        TruckRoute(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments,
            const std::vector<double> &waiting_time_violations,
            const double &distance,
            const double &weight)
            : _BaseRoute(customers, time_segments, waiting_time_violations, distance, weight) {}

        /** @brief Construct a `TruckRoute` with pre-calculated time_segments */
        TruckRoute(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments)
            : TruckRoute(
                  customers,
                  time_segments,
                  _calculate_waiting_time_violations(customers, time_segments),
                  _calculate_distance(customers),
                  _calculate_weight(customers)) {}

        /** @brief Construct a `TruckRoute` from a list of customers in order. */
        TruckRoute(const std::vector<std::size_t> &customers)
            : TruckRoute(
                  customers,
                  _calculate_time_segments(customers)) {}

        double weight_violation() const override
        {
            auto problem = Problem::get_instance();
            return std::max(0.0, weight - problem->truck->capacity);
        }
    };

    utils::FenwickTree<double> TruckRoute::_calculate_time_segments(const std::vector<std::size_t> &customers)
    {
        auto problem = Problem::get_instance();
        utils::FenwickTree<double> time_segments;

        std::size_t coefficients_index = 0;
        double current_within_timespan = 0;
        const auto shift = [&coefficients_index, &current_within_timespan](double *time_segment_ptr, double dt)
        {
            *time_segment_ptr += dt;
            current_within_timespan += dt;
            if (current_within_timespan >= 3600)
            {
                current_within_timespan -= 3600;
                coefficients_index++;
            }
        };

        time_segments.reserve(customers.size() - 1);
        for (std::size_t i = 0; i + 1 < customers.size(); i++)
        {
            double time_segment = 0, distance = problem->distances[customers[i]][customers[i + 1]];

            shift(&time_segment, problem->customers[customers[i]].truck_service_time);
            while (distance > 0)
            {
                double speed = problem->truck->speed(coefficients_index),
                       distance_shift = std::min(distance, speed * (3600.0 - current_within_timespan));

                distance -= distance_shift;
                shift(&time_segment, distance_shift / speed);
            }

            time_segments.push_back(time_segment);
        }

        return time_segments;
    }

    std::vector<double> TruckRoute::_calculate_waiting_time_violations(
        const std::vector<std::size_t> &customers,
        const utils::FenwickTree<double> &time_segments)
    {
        auto problem = Problem::get_instance();
        return _BaseRoute::_calculate_waiting_time_violations(
            customers,
            time_segments,
            [&problem](const std::size_t &customer)
            {
                return problem->customers[customer].truck_service_time;
            });
    }

    /** @brief Represents a drone route. */
    class DroneRoute : public _BaseRoute
    {
    private:
        static utils::FenwickTree<double> _calculate_time_segments(const std::vector<std::size_t> &customers);
        static std::vector<double> _calculate_waiting_time_violations(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments);
        static double _calculate_energy_consumption(const std::vector<std::size_t> &customers);

    public:
        /** @brief Total energy consumption of drone (SI unit: J) */
        const double energy_consumption;

        /** @brief Construct a `DroneRoute` with pre-calculated attributes */
        DroneRoute(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments,
            const std::vector<double> &waiting_time_violations,
            const double &distance,
            const double &weight,
            const double &energy_consumption)
            : _BaseRoute(customers, time_segments, waiting_time_violations, distance, weight),
              energy_consumption(energy_consumption) {}

        /** @brief Construct a `DroneRoute` with pre-calculated time_segments */
        DroneRoute(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments)
            : DroneRoute(
                  customers,
                  time_segments,
                  _calculate_waiting_time_violations(customers, time_segments),
                  _calculate_distance(customers),
                  _calculate_weight(customers),
                  _calculate_energy_consumption(customers)) {}

        /** @brief Construct a `DroneRoute` from a list of customers in order. */
        DroneRoute(const std::vector<std::size_t> &customers)
            : DroneRoute(customers, _calculate_time_segments(customers)) {}

        double weight_violation() const override
        {
            auto problem = Problem::get_instance();
            return std::max(0.0, weight - problem->drone->capacity);
        }
    };

    utils::FenwickTree<double> DroneRoute::_calculate_time_segments(const std::vector<std::size_t> &customers)
    {
        auto problem = Problem::get_instance();
        utils::FenwickTree<double> time_segments;

        auto drone = problem->drone;
        time_segments.reserve(customers.size() - 1);
        for (std::size_t i = 0; i + 1 < customers.size(); i++)
        {
            time_segments.push_back(
                problem->customers[customers[i]].drone_service_time +
                drone->takeoff_time() +
                drone->cruise_time(problem->distances[customers[i]][customers[i + 1]]) +
                drone->landing_time());
        }

        return time_segments;
    }

    std::vector<double> DroneRoute::_calculate_waiting_time_violations(
        const std::vector<std::size_t> &customers,
        const utils::FenwickTree<double> &time_segments)
    {
        auto problem = Problem::get_instance();
        return _BaseRoute::_calculate_waiting_time_violations(
            customers,
            time_segments,
            [&problem](const std::size_t &customer)
            {
                return problem->customers[customer].drone_service_time;
            });
    }

    double DroneRoute::_calculate_energy_consumption(const std::vector<std::size_t> &customers)
    {
        auto problem = Problem::get_instance();
        double energy = 0, weight = 0;

        auto drone = problem->drone;
        for (std::size_t i = 0; i + 1 < customers.size(); i++)
        {
            weight += problem->customers[customers[i]].demand;
            energy += drone->takeoff_time() * drone->takeoff_power(weight) +
                      drone->landing_time() * drone->landing_power(weight) +
                      drone->cruise_time(problem->distances[customers[i]][customers[i + 1]]) * drone->cruise_power(weight);
        }

        return energy;
    }
}
