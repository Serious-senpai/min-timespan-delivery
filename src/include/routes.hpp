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

    public:
        static std::vector<double> calculate_waiting_time_violations(
            const std::vector<std::size_t> &customers,
            const std::vector<double> &time_segments,
            const std::function<double(const std::size_t &)> service_time);

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

    std::vector<double> _BaseRoute::calculate_waiting_time_violations(
        const std::vector<std::size_t> &customers,
        const std::vector<double> &time_segments,
        const std::function<double(const std::size_t &)> service_time)
    {
        auto problem = Problem::get_instance();
        std::vector<double> violations(customers.size());

        double time = std::accumulate(time_segments.begin(), time_segments.end(), 0.0);
        for (std::size_t i = 0; i < customers.size(); i++)
        {
            violations[i] = std::max(0.0, time - service_time(customers[i]) - problem->maximum_waiting_time);
            time -= time_segments[i];
        }

        violations.front() = violations.back() = 0;

        return violations;
    }

    /** @brief Represents a truck route. */
    class TruckRoute : public _BaseRoute
    {
    public:
        static std::vector<double> calculate_time_segments(
            const std::vector<std::size_t> &customers,
            std::size_t &coefficients_index,
            double &current_within_timespan);
        static std::vector<double> calculate_waiting_time_violations(
            const std::vector<std::size_t> &customers,
            const std::vector<double> &time_segments);

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
            std::vector<std::size_t> new_customers(_customers);
            new_customers.insert(new_customers.end() - 1, customer);
            *this = TruckRoute(new_customers);
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
            if (current_within_timespan >= ONE_HOUR)
            {
                current_within_timespan -= ONE_HOUR;
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
                       distance_shift = std::min(distance, speed * (ONE_HOUR - current_within_timespan));

                distance -= distance_shift;
                shift(&time_segment, distance_shift / speed);
            }

            time_segments.push_back(time_segment);
        }

        return time_segments;
    }

    std::vector<double> TruckRoute::calculate_waiting_time_violations(
        const std::vector<std::size_t> &customers,
        const std::vector<double> &time_segments)
    {
        auto problem = Problem::get_instance();
        return _BaseRoute::calculate_waiting_time_violations(
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
        static std::vector<double> _calculate_time_segments(const std::vector<std::size_t> &customers);
        static std::vector<double> _calculate_waiting_time_violations(
            const std::vector<std::size_t> &customers,
            const std::vector<double> &time_segments);
        static double _calculate_energy_consumption(const std::vector<std::size_t> &customers);
        static double _calculate_fixed_time_violation(const std::vector<double> &time_segments);

        std::vector<double> _time_segments;
        std::vector<double> _waiting_time_violations;
        double _working_time;
        double _energy_consumption;
        double _fixed_time_violation;

    public:
        /** @brief Construct a `DroneRoute` with pre-calculated attributes. */
        DroneRoute(
            const std::vector<std::size_t> &customers,
            const std::vector<double> &time_segments,
            const std::vector<double> &waiting_time_violations,
            const double &distance,
            const double &weight,
            const double &energy_consumption,
            const double &fixed_time_violation)
            : _BaseRoute(customers, distance, weight),
              _time_segments(time_segments),
              _waiting_time_violations(waiting_time_violations),
              _working_time(std::accumulate(time_segments.begin(), time_segments.end(), 0.0)),
              _energy_consumption(energy_consumption),
              _fixed_time_violation(fixed_time_violation)
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
                  _calculate_waiting_time_violations(customers, time_segments),
                  distance,
                  weight,
                  energy_consumption,
                  _calculate_fixed_time_violation(time_segments)) {}

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
         * @brief The waiting time violations of each customer in this route.
         */
        const std::vector<double> &waiting_time_violations() const
        {
            return _waiting_time_violations;
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

        /**
         * @brief Append a new customer to this route.
         *
         * This is a convenient method to use extensively during algorithm initialization step.
         */
        void push_back(const std::size_t &customer)
        {
            std::vector<std::size_t> new_customers(_customers);
            new_customers.insert(new_customers.end() - 1, customer);
            *this = DroneRoute(new_customers);
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

    std::vector<double> DroneRoute::_calculate_waiting_time_violations(
        const std::vector<std::size_t> &customers,
        const std::vector<double> &time_segments)
    {
        auto problem = Problem::get_instance();
        return _BaseRoute::calculate_waiting_time_violations(
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

    double DroneRoute::_calculate_fixed_time_violation(const std::vector<double> &time_segments)
    {
        auto problem = Problem::get_instance();
        if (problem->endurance != nullptr)
        {
            return std::max(0.0, std::accumulate(time_segments.begin(), time_segments.end(), 0.0) - problem->endurance->fixed_time);
        }

        return 0;
    }

    template <typename T, typename... Args>
    struct is_route
    {
        constexpr static bool value = std::conjunction_v<std::disjunction<std::is_same<T, TruckRoute>, std::is_same<T, DroneRoute>>, is_route<Args>...>;
    };

    template <typename T, typename... Args>
    constexpr bool is_route_v = is_route<T, Args...>::value;

    template <typename _RT, std::enable_if_t<is_route_v<_RT>, bool> = true>
    bool operator==(const _RT &f, const _RT &s)
    {
        return f.customers() == s.customers();
    }
}

namespace std
{
    ostream &operator<<(ostream &stream, const d2d::_BaseRoute &route)
    {
        return stream << route.customers();
    }
}
