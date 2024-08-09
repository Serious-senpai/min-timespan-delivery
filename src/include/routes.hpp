#pragma once

#include "errors.hpp"
#include "problem.hpp"

namespace d2d
{
    class _BaseRoute
    {
    protected:
        static double _calculate_distance(const std::vector<std::size_t> &customers);
        static double _calculate_weight(const std::vector<std::size_t> &customers);

        std::vector<std::size_t> _customers;
        double _distance;
        double _weight;

        _BaseRoute(
            const std::vector<std::size_t> &customers,
            const double &distance,
            const double &weight)
            : _customers(customers),
              _distance(distance),
              _weight(weight)
        {
#ifdef DEBUG
            if (customers.size() < 3)
            {
                throw std::runtime_error("Empty routes are not allowed");
            }

            if (customers.front() != 0 || customers.back() != 0)
            {
                throw std::runtime_error("Routes must start and end at the depot");
            }
#endif
        }

        template <typename T, std::enable_if_t<std::is_base_of_v<_BaseRoute, T>, bool> = true>
        void _verify(const T &verify) const
        {
#ifdef DEBUG
            if (!utils::approximate(_distance, verify._distance))
            {
                throw std::runtime_error("Inconsistent distance, possibly an error in calculation");
            }

            if (!utils::approximate(_weight, verify._weight))
            {
                throw std::runtime_error("Inconsistent weight, possibly an error in calculation");
            }
#endif
        }

        template <typename T, std::enable_if_t<std::conjunction_v<std::is_base_of<_BaseRoute, T>, std::is_constructible<T, const std::vector<std::size_t> &>>, bool> = true>
        void _verify() const
        {
            _verify<T>(T(_customers));
        }

    public:
        /** @brief The amount of weight exceeding vehicle capacity. */
        virtual double capacity_violation() const = 0;

        /**
         * @brief The order of customers in this route, starting and ending at the depot `0`.
         */
        const std::vector<std::size_t> &customers() const
        {
            return _customers;
        }

        /**
         * @brief The total traveling distance of this route.
         */
        double distance() const
        {
            return _distance;
        }

        /**
         * @brief The total customer demands of this route.
         */
        double weight() const
        {
            return _weight;
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

    /** @brief Represents a truck route. */
    class TruckRoute : public _BaseRoute
    {
    protected:
        void _verify()
        {
#ifdef DEBUG
            _BaseRoute::_verify<TruckRoute>();
#endif
        }

    public:
        static std::vector<double> calculate_time_segments(
            const std::vector<std::size_t> &customers,
            std::size_t &coefficients_index,
            double &current_within_timespan);

        /** @brief Construct a `TruckRoute` with pre-calculated attributes */
        TruckRoute(
            const std::vector<std::size_t> &customers,
            const double &distance,
            const double &weight)
            : _BaseRoute(customers, distance, weight) {}

        /** @brief Construct a `TruckRoute` from a list of customers in order. */
        TruckRoute(const std::vector<std::size_t> &customers)
            : TruckRoute(customers, _calculate_distance(customers), _calculate_weight(customers)) {}

        double capacity_violation() const override
        {
            auto problem = Problem::get_instance();
            return std::max(0.0, _weight - problem->truck->capacity);
        }

        /**
         * @brief Append a new customer to this route.
         *
         * This is a convenient method to use extensively during algorithm initialization step.
         */
        void push_back(const std::size_t &customer)
        {
            auto problem = Problem::get_instance();

            _customers.back() = customer;
            _customers.push_back(0); // Done updating _customers

            std::size_t old_last_index = _customers.size() - 3;
            _distance += problem->distances[_customers[old_last_index]][_customers[old_last_index + 1]] +
                         problem->distances[_customers[old_last_index + 1]][0] -
                         problem->distances[_customers[old_last_index]][0]; // Done updating _distance
            _weight += problem->customers[customer].demand;                 // Done updating _weight

            _verify();
        }
    };

    std::vector<double> TruckRoute::calculate_time_segments(
        const std::vector<std::size_t> &customers,
        std::size_t &coefficients_index,
        double &current_within_timespan)
    {
        auto problem = Problem::get_instance();
        std::vector<double> time_segments;

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

    /** @brief Represents a drone route. */
    class DroneRoute : public _BaseRoute
    {
    private:
        static std::vector<double> _calculate_time_segments(const std::vector<std::size_t> &customers);
        static double _calculate_energy_consumption(const std::vector<std::size_t> &customers);
        static double _calculate_fixed_time_violation(const std::vector<double> &time_segments);
        static double _calculate_fixed_distance_violation(const double &distance);

        std::vector<double> _time_segments;
        double _working_time;
        double _energy_consumption;
        double _fixed_time_violation;
        double _fixed_distance_violation;

    protected:
        void _verify()
        {
#ifdef DEBUG
            DroneRoute verify(_customers);
            _BaseRoute::_verify<DroneRoute>(verify);
            if (!utils::approximate(_time_segments, verify._time_segments))
            {
                throw std::runtime_error("Inconsistent time segments, possibly an error in calculation");
            }

            if (!utils::approximate(_working_time, verify._working_time))
            {
                throw std::runtime_error("Inconsistent working time, possibly an error in calculation");
            }

            if (!utils::approximate(_energy_consumption, verify._energy_consumption))
            {
                throw std::runtime_error("Inconsistent energy consumption, possibly an error in calculation");
            }
#endif
        }

    public:
        /** @brief Construct a `DroneRoute` with pre-calculated attributes. */
        DroneRoute(
            const std::vector<std::size_t> &customers,
            const std::vector<double> &time_segments,
            const double &distance,
            const double &weight,
            const double &energy_consumption,
            const double &fixed_time_violation,
            const double &fixed_distance_violation)
            : _BaseRoute(customers, distance, weight),
              _time_segments(time_segments),
              _working_time(std::accumulate(time_segments.begin(), time_segments.end(), 0.0)),
              _energy_consumption(energy_consumption),
              _fixed_time_violation(fixed_time_violation),
              _fixed_distance_violation(fixed_distance_violation)
        {
#ifdef DEBUG
            auto problem = Problem::get_instance();
            for (auto &customer : customers)
            {
                if (!problem->customers[customer].dronable)
                {
                    throw NonDronable(customer);
                }
            }
#endif
        }

        /**
         * @brief Construct a `DroneRoute` with pre-calculated `time_segments`, `distance`, `weight`
         * and `energy_consumption`.
         */
        DroneRoute(
            const std::vector<std::size_t> &customers,
            const std::vector<double> &time_segments,
            const double &distance,
            const double &weight,
            const double &energy_consumption)
            : DroneRoute(
                  customers,
                  time_segments,
                  distance,
                  weight,
                  energy_consumption,
                  _calculate_fixed_time_violation(time_segments),
                  _calculate_fixed_distance_violation(distance)) {}

        /** @brief Construct a `DroneRoute` with pre-calculated `time_segments`. */
        DroneRoute(
            const std::vector<std::size_t> &customers,
            const std::vector<double> &time_segments)
            : DroneRoute(
                  customers,
                  time_segments,
                  _calculate_distance(customers),
                  _calculate_weight(customers),
                  _calculate_energy_consumption(customers)) {}

        /** @brief Construct a `DroneRoute` with pre-calculated `distance`, `weight` and `energy_consumption`. */
        DroneRoute(
            const std::vector<std::size_t> &customers,
            const double &distance,
            const double &weight,
            const double &energy_consumption)
            : DroneRoute(
                  customers,
                  _calculate_time_segments(customers),
                  distance,
                  weight,
                  energy_consumption) {}

        /** @brief Construct a `DroneRoute` from a list of customers in order. */
        DroneRoute(const std::vector<std::size_t> &customers)
            : DroneRoute(customers, _calculate_time_segments(customers)) {}

        /**
         * @brief The time segments between each pair of adjacent customers.
         *
         * Given a pair of adjacent customers `(x, y)` in this route, a time segment is the time
         * from the moment the vehicle starts serving customer `x` to the moment it starts serving
         * customer `y` (including service time of `x` but not `y`).
         */
        const std::vector<double> &time_segments() const
        {
            return _time_segments;
        }

        /**
         * @brief The total working time of this route
         */
        double working_time() const
        {
            return _working_time;
        }

        double capacity_violation() const override
        {
            auto problem = Problem::get_instance();
            return std::max(0.0, _weight - problem->drone->capacity);
        }

        /** @brief Total energy consumption of drone (SI unit: J) */
        double energy_consumption() const
        {
            return _energy_consumption;
        }

        double energy_violation() const
        {
            auto problem = Problem::get_instance();
            if (problem->linear != nullptr)
            {
                return std::max(0.0, _energy_consumption - problem->linear->battery);
            }
            else if (problem->nonlinear != nullptr)
            {
                return std::max(0.0, _energy_consumption - problem->nonlinear->battery);
            }

            return 0;
        }

        double fixed_time_violation() const
        {
            return _fixed_time_violation;
        }

        double fixed_distance_violation() const
        {
            return _fixed_distance_violation;
        }

        /**
         * @brief Append a new customer to this route.
         *
         * This is a convenient method to use extensively during algorithm initialization step.
         */
        void push_back(const std::size_t &customer)
        {
            auto problem = Problem::get_instance();
#ifdef DEBUG
            if (!problem->customers[customer].dronable)
            {
                throw NonDronable(customer);
            }
#endif

            auto drone = problem->drone;

            _customers.back() = customer;
            _customers.push_back(0); // Done updating _customers

            _time_segments.pop_back();

            std::size_t old_last_index = _customers.size() - 3;

            _distance -= problem->distances[_customers[old_last_index]][0];
            _energy_consumption -= drone->takeoff_time() * drone->takeoff_power(_weight) +
                                   drone->landing_time() * drone->landing_power(_weight) +
                                   drone->cruise_time(problem->distances[_customers[old_last_index]][0]) * drone->cruise_power(_weight);
            _weight -= problem->customers[_customers[old_last_index]].demand;

            for (std::size_t i = old_last_index; i + 1 < _customers.size(); i++)
            {
                double distance = problem->distances[_customers[i]][_customers[i + 1]];
                _time_segments.push_back(
                    problem->customers[_customers[i]].drone_service_time +
                    drone->takeoff_time() +
                    drone->cruise_time(distance) +
                    drone->landing_time());

                _distance += distance;
                _weight += problem->customers[_customers[i]].demand;
                _energy_consumption += drone->takeoff_time() * drone->takeoff_power(_weight) +
                                       drone->cruise_time(distance) * drone->cruise_power(_weight) +
                                       drone->landing_time() * drone->landing_power(_weight);
            } // Done updating _time_segments, _distance, _weight, _energy_consumption

            _working_time = std::accumulate(_time_segments.begin(), _time_segments.end(), 0.0); // Done updating _working_time

            _verify();
        }
    };

    std::vector<double> DroneRoute::_calculate_time_segments(const std::vector<std::size_t> &customers)
    {
        auto problem = Problem::get_instance();
        std::vector<double> time_segments;

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

    double DroneRoute::_calculate_energy_consumption(const std::vector<std::size_t> &customers)
    {
        auto problem = Problem::get_instance();
        double energy = 0, weight = 0;

        auto drone = problem->drone;
        for (std::size_t i = 0; i + 1 < customers.size(); i++)
        {
            weight += problem->customers[customers[i]].demand;
            energy += drone->takeoff_time() * drone->takeoff_power(weight) +
                      drone->cruise_time(problem->distances[customers[i]][customers[i + 1]]) * drone->cruise_power(weight) +
                      drone->landing_time() * drone->landing_power(weight);
        }

        return energy;
    }

    double DroneRoute::_calculate_fixed_time_violation(const std::vector<double> &time_segments)
    {
        auto problem = Problem::get_instance();
        if (problem->endurance != nullptr)
        {
            return std::max(0.0, std::accumulate(time_segments.begin(), time_segments.end(), 0.0) - problem->endurance->fixed_time);
        }

        return 0;
    }

    double DroneRoute::_calculate_fixed_distance_violation(const double &distance)
    {
        auto problem = Problem::get_instance();
        if (problem->endurance != nullptr)
        {
            return std::max(0.0, distance - problem->endurance->fixed_distance);
        }

        return 0;
    }
}

namespace std
{
    ostream &operator<<(ostream &stream, const d2d::_BaseRoute &route)
    {
        return stream << route.customers();
    }
}
