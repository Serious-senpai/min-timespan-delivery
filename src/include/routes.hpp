#pragma once

#include "errors.hpp"
#include "fenwick.hpp"
#include "problem.hpp"

namespace d2d
{
    class _BaseRoute
    {
    protected:
        static double _calculate_distance(const std::vector<std::size_t> &customers);
        static double _calculate_weight(const std::vector<std::size_t> &customers);
        static utils::FenwickTree<double> _calculate_waiting_time_violations(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments,
            const std::function<double(const std::size_t &)> service_time);

        std::vector<std::size_t> _customers;
        utils::FenwickTree<double> _time_segments;
        utils::FenwickTree<double> _waiting_time_violations;
        double _distance;
        double _weight;
        double _working_time;

        _BaseRoute(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments,
            const utils::FenwickTree<double> &waiting_time_violations,
            const double &distance,
            const double &weight)
            : _customers(customers),
              _time_segments(time_segments),
              _waiting_time_violations(waiting_time_violations),
              _distance(distance),
              _weight(weight),
              _working_time(time_segments.sum())
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
            if (!utils::approximate(_time_segments, verify._time_segments))
            {
                throw std::runtime_error("Inconsistent time segments, possibly an error in calculation");
            }

            if (!utils::approximate(_waiting_time_violations, verify._waiting_time_violations))
            {
                throw std::runtime_error("Inconsistent waiting time violations, possibly an error in calculation");
            }

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

        template <typename T, std::enable_if_t<std::is_base_of_v<_BaseRoute, T> && std::is_constructible_v<T, const std::vector<std::size_t> &>, bool> = true>
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
         * @brief The time segments between each pair of adjacent customers.
         *
         * Given a pair of adjacent customers `(x, y)` in this route, a time segment is the time
         * from the moment the vehicle starts serving customer `x` to the moment it starts serving
         * customer `y` (including service time of `x` but not `y`).
         */
        const utils::FenwickTree<double> &time_segments() const
        {
            return _time_segments;
        }

        /**
         * @brief The waiting time violations of each customer in this route.
         */
        const utils::FenwickTree<double> &waiting_time_violations() const
        {
            return _waiting_time_violations;
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

        /**
         * @brief The total working time of this route
         */
        double working_time() const
        {
            return _working_time;
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

    utils::FenwickTree<double> _BaseRoute::_calculate_waiting_time_violations(
        const std::vector<std::size_t> &customers,
        const utils::FenwickTree<double> &time_segments,
        const std::function<double(const std::size_t &)> service_time)
    {
        auto problem = Problem::get_instance();
        utils::FenwickTree<double> violations;
        violations.reserve(customers.size());

        double time = time_segments.sum();
        for (std::size_t i = 0; i < customers.size(); i++)
        {
            violations.push_back(std::max(0.0, time - service_time(customers[i]) - problem->maximum_waiting_time));
            time -= time_segments.get(i);
        }

        return violations;
    }

    /** @brief Represents a truck route. */
    class TruckRoute : public _BaseRoute
    {
    private:
        static utils::FenwickTree<double> _calculate_time_segments(const std::vector<std::size_t> &customers);
        static utils::FenwickTree<double> _calculate_waiting_time_violations(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments);

    protected:
        void _verify()
        {
#ifdef DEBUG
            _BaseRoute::_verify<TruckRoute>();
#endif
        }

    public:
        /** @brief Construct a `TruckRoute` with pre-calculated attributes */
        TruckRoute(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments,
            const utils::FenwickTree<double> &waiting_time_violations,
            const double &distance,
            const double &weight)
            : _BaseRoute(customers, time_segments, waiting_time_violations, distance, weight) {}

        /**
         * @brief Construct a `TruckRoute` with pre-calculated `time_segments`, `distance` and `weight`.
         */
        TruckRoute(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments,
            const double &distance,
            const double &weight)
            : TruckRoute(
                  customers,
                  time_segments,
                  _calculate_waiting_time_violations(customers, time_segments),
                  distance,
                  weight) {}

        /** @brief Construct a `TruckRoute` with pre-calculated time_segments */
        TruckRoute(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments)
            : TruckRoute(
                  customers,
                  time_segments,
                  _calculate_distance(customers),
                  _calculate_weight(customers)) {}

        /** @brief Construct a `TruckRoute` from a list of customers in order. */
        TruckRoute(const std::vector<std::size_t> &customers)
            : TruckRoute(customers, _calculate_time_segments(customers)) {}

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

            _time_segments.pop_back();

            double time_offset = _time_segments.sum();
            std::size_t coefficients_index = time_offset / 3600.0;
            double current_within_timespan = time_offset - 3600.0 * coefficients_index;

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

            std::size_t old_last_index = _customers.size() - 3;
            _distance -= problem->distances[_customers[old_last_index]][0];

            for (std::size_t i = old_last_index; i + 1 < _customers.size(); i++)
            {
                double time_segment = 0,
                       distance = problem->distances[_customers[i]][_customers[i + 1]];
                shift(&time_segment, problem->customers[_customers[i]].truck_service_time);

                _distance += distance;
                while (distance > 0)
                {
                    double speed = problem->truck->speed(coefficients_index),
                           distance_shift = std::min(distance, speed * (3600.0 - current_within_timespan));

                    distance -= distance_shift;
                    shift(&time_segment, distance_shift / speed);
                }

                _time_segments.push_back(time_segment);
            } // Done updating _time_segments, _distance

            _weight += problem->customers[customer].demand; // Done updating _weight

            // Couldn't find a better way than recalculating it
            _waiting_time_violations = _calculate_waiting_time_violations(_customers, _time_segments); // Done updating _waiting_time_violations

            _verify();
        }

        void reverse(const std::size_t &offset, const std::size_t &length)
        {
            if (length < 2)
            {
                return;
            }

            auto problem = Problem::get_instance();

            std::reverse(_customers.begin() + offset, _customers.begin() + (offset + length)); // Done updating _customers

            // Too lazy to implement recalculation, still O(nlogn) though.
            // Algorithm complexity doesn't even matter in the first place - typically n < 20
            _time_segments = _calculate_time_segments(_customers); // Done updating _time_segments

            _distance += problem->distances[_customers[offset - 1]][_customers[offset]] +
                         problem->distances[_customers[offset + length - 1]][_customers[offset + length]] -
                         problem->distances[_customers[offset - 1]][_customers[offset + length - 1]] -
                         problem->distances[_customers[offset]][_customers[offset + length]]; // Done updating _distance

            // _weight = _weight; // Unchanged, done updating _weight

            _waiting_time_violations = _calculate_waiting_time_violations(_customers, _time_segments); // Done updating _waiting_time_violations

            _verify();
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

    utils::FenwickTree<double> TruckRoute::_calculate_waiting_time_violations(
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
        static utils::FenwickTree<double> _calculate_waiting_time_violations(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments);
        static double _calculate_energy_consumption(const std::vector<std::size_t> &customers);

        double _energy_consumption;

    protected:
        void _verify()
        {
#ifdef DEBUG
            DroneRoute verify(_customers);
            _BaseRoute::_verify<DroneRoute>(verify);
            if (!utils::approximate(_energy_consumption, verify._energy_consumption))
            {
                throw std::runtime_error("DroneRoute::push_back: Inconsistent energy consumption, possibly an error in calculation");
            }
#endif
        }

    public:
        /** @brief Construct a `DroneRoute` with pre-calculated attributes. */
        DroneRoute(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments,
            const utils::FenwickTree<double> &waiting_time_violations,
            const double &distance,
            const double &weight,
            const double &energy_consumption)
            : _BaseRoute(customers, time_segments, waiting_time_violations, distance, weight),
              _energy_consumption(energy_consumption)
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
            const utils::FenwickTree<double> &time_segments,
            const double &distance,
            const double &weight,
            const double &energy_consumption)
            : DroneRoute(
                  customers,
                  time_segments,
                  _calculate_waiting_time_violations(customers, time_segments),
                  distance,
                  weight,
                  energy_consumption) {}

        /** @brief Construct a `DroneRoute` with pre-calculated `time_segments`. */
        DroneRoute(
            const std::vector<std::size_t> &customers,
            const utils::FenwickTree<double> &time_segments)
            : DroneRoute(
                  customers,
                  time_segments,
                  _calculate_distance(customers),
                  _calculate_weight(customers),
                  _calculate_energy_consumption(customers)) {}

        /** @brief Construct a `DroneRoute` from a list of customers in order. */
        DroneRoute(const std::vector<std::size_t> &customers)
            : DroneRoute(customers, _calculate_time_segments(customers)) {}

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

        /**
         * @brief Append a new customer to this route.
         *
         * This is a convenient method to use extensively during algorithm initialization step.
         */
        void push_back(const std::size_t &customer)
        {
            auto problem = Problem::get_instance();
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

            _waiting_time_violations = _calculate_waiting_time_violations(_customers, _time_segments); // Done updating _waiting_time_violations

            _verify();
        }

        void reverse(const std::size_t &offset, const std::size_t &length)
        {
            if (length < 2)
            {
                return;
            }

            auto problem = Problem::get_instance();

            std::reverse(_customers.begin() + offset, _customers.begin() + (offset + length)); // Done updating _customers

            _time_segments = _calculate_time_segments(_customers); // Done updating _time_segments

            _distance += problem->distances[_customers[offset - 1]][_customers[offset]] +
                         problem->distances[_customers[offset + length - 1]][_customers[offset + length]] -
                         problem->distances[_customers[offset - 1]][_customers[offset + length - 1]] -
                         problem->distances[_customers[offset]][_customers[offset + length]]; // Done updating _distance

            _energy_consumption = _calculate_energy_consumption(_customers); // Done updating _energy_consumption

            // _weight = _weight; // Unchanged, done updating _weight

            _waiting_time_violations = _calculate_waiting_time_violations(_customers, _time_segments); // Done updating _waiting_time_violations

            _verify();
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

    utils::FenwickTree<double> DroneRoute::_calculate_waiting_time_violations(
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
                      drone->cruise_time(problem->distances[customers[i]][customers[i + 1]]) * drone->cruise_power(weight) +
                      drone->landing_time() * drone->landing_power(weight);
        }

        return energy;
    }
}

namespace std
{
    ostream &operator<<(ostream &stream, const d2d::_BaseRoute &route)
    {
        return stream << route.customers();
    }
}
