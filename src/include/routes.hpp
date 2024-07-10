#pragma once

#include "fenwick.hpp"
#include "problem.hpp"

namespace d2d
{
    class _BaseRoute
    {
    protected:
        static double _calculate_distance(const std::vector<std::size_t> &customers);
        static utils::FenwickTree<double> _calculate_weights(const std::vector<std::size_t> &customers);

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
         * @brief The total traveling distance of this route.
         */
        const double distance;

        /**
         * @brief A `FenwickTree` containing the weight of each customer in this route.
         */
        const utils::FenwickTree<double> weights;

        _BaseRoute(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments,
            const utils::FenwickTree<double> &weights,
            const double distance)
            : customers(customers),
              time_segments(time_segments),
              weights(weights),
              distance(distance)
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

    utils::FenwickTree<double> _BaseRoute::_calculate_weights(const std::vector<std::size_t> &customers)
    {
        auto problem = Problem::get_instance();
        utils::FenwickTree<double> weights;
        for (std::size_t i = 1; i < customers.size(); i++)
        {
            weights.push_back(problem->customers[customers[i]].demand);
        }

        return weights;
    }

    /** @brief Represents a truck route. */
    class TruckRoute : public _BaseRoute
    {
    private:
        static utils::FenwickTree<double> _calculate_time_segments(const std::vector<std::size_t> &customers);

    public:
        /** @brief Construct a `TruckRoute` with pre-calculated attributes */
        TruckRoute(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments,
            const utils::FenwickTree<double> &weights,
            const double distance)
            : _BaseRoute(customers, time_segments, weights, distance) {}

        /** @brief Construct a `TruckRoute` from a list of customers in order. */
        TruckRoute(const std::vector<std::size_t> &customers)
            : TruckRoute(customers, _calculate_time_segments(customers), _calculate_weights(customers), _calculate_distance(customers)) {}
    };

    utils::FenwickTree<double> TruckRoute::_calculate_time_segments(const std::vector<std::size_t> &customers)
    {
        auto problem = Problem::get_instance();
        utils::FenwickTree<double> time_segments;
        double time_segment = 0;

        std::size_t coefficients_index = 0;
        double current_within_timespan = 0;
        const auto shift = [&time_segment, &coefficients_index, &current_within_timespan](double dt)
        {
            time_segment += dt;
            current_within_timespan += dt;
            if (current_within_timespan >= 3600)
            {
                current_within_timespan -= 3600;
                coefficients_index++;
            }
        };

        for (std::size_t i = 0; i + 1 < customers.size(); i++)
        {
            double distance = problem->distances[customers[i]][customers[i + 1]],
                   max_speed = problem->truck->maximum_velocity;

            shift(problem->customers[customers[i]].truck_service_time);
            while (distance > 0)
            {
                double speed = max_speed * problem->truck->coefficients[coefficients_index % problem->truck->coefficients.size()],
                       distance_shift = std::min(distance, speed * (3600.0 - current_within_timespan));

                distance -= distance_shift;
                shift(distance_shift / speed);
            }

            time_segments.push_back(time_segment);
            time_segment = 0;
        }

        return time_segments;
    }

    /** @brief Represents a drone route. */
    class DroneRoute : public _BaseRoute
    {
    private:
        static utils::FenwickTree<double> _calculate_time_segments(const std::vector<std::size_t> &customers);

    public:
        /** @brief Construct a `DroneRoute` with pre-calculated attributes */
        DroneRoute(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments,
            const utils::FenwickTree<double> &weights,
            const double distance)
            : _BaseRoute(customers, time_segments, weights, distance) {}

        /** @brief Construct a `DroneRoute` from a list of customers in order. */
        DroneRoute(const std::vector<std::size_t> &customers)
            : DroneRoute(customers, _calculate_time_segments(customers), _calculate_weights(customers), _calculate_distance(customers)) {}
    };

    utils::FenwickTree<double> DroneRoute::_calculate_time_segments(const std::vector<std::size_t> &customers)
    {
        auto problem = Problem::get_instance();
        utils::FenwickTree<double> time_segments;

        for (std::size_t i = 0; i + 1 < customers.size(); i++)
        {
            time_segments.push_back(
                problem->customers[customers[i]].drone_service_time +
                problem->drone->takeoff_time() +
                problem->drone->cruise_power(problem->distances[customers[i]][customers[i + 1]]) +
                problem->drone->landing_time());
        }

        return time_segments;
    }
}
