#pragma once

#include "standard.hpp"
#include "utils.hpp"

#define ONE_HOUR 60.0

namespace d2d
{
    class TruckConfig
    {
    private:
        const double _maximum_velocity;
        const std::vector<double> _coefficients;

    public:
        const double capacity;
        const double average_speed;

        TruckConfig(
            const double &maximum_velocity,
            const std::vector<double> &coefficients,
            const double &capacity)
            : _maximum_velocity(maximum_velocity),
              _coefficients(coefficients),
              capacity(capacity),
              average_speed(maximum_velocity * std::accumulate(coefficients.begin(), coefficients.end(), 0.0) / coefficients.size())
        {
        }

        double speed(const std::size_t &index) const
        {
            return _maximum_velocity * _coefficients[index % _coefficients.size()];
        }
    };

    enum StatsType
    {
        low,
        high
    };

    class _BaseDroneConfig
    {
    public:
        const double capacity;
        const StatsType speed_type;
        const StatsType range_type;

        _BaseDroneConfig(
            const double capacity,
            const StatsType speed_type,
            const StatsType range_type)
            : capacity(capacity),
              speed_type(speed_type),
              range_type(range_type)
        {
        }

        virtual ~_BaseDroneConfig() = default;

        virtual double takeoff_power(const double weight) const = 0;
        virtual double landing_power(const double weight) const = 0;
        virtual double cruise_power(const double weight) const = 0;

        virtual double takeoff_time() const = 0;
        virtual double landing_time() const = 0;
        virtual double cruise_time(const double distance) const = 0;
    };

    class _VariableDroneConfig : public _BaseDroneConfig
    {
    public:
        const double takeoff_speed;
        const double cruise_speed;
        const double landing_speed;
        const double altitude;
        const double battery;

        _VariableDroneConfig(
            const double capacity,
            const StatsType speed_type,
            const StatsType range_type,
            const double takeoff_speed,
            const double cruise_speed,
            const double landing_speed,
            const double altitude,
            const double battery)
            : _BaseDroneConfig(capacity, speed_type, range_type),
              takeoff_speed(takeoff_speed),
              cruise_speed(cruise_speed),
              landing_speed(landing_speed),
              altitude(altitude),
              battery(battery)
        {
        }

        virtual ~_VariableDroneConfig() = default;

        double takeoff_time() const override
        {
            return altitude / takeoff_speed;
        }

        double landing_time() const override
        {
            return altitude / landing_speed;
        }

        double cruise_time(const double distance) const override
        {
            return distance / cruise_speed;
        }
    };

    class DroneLinearConfig : public _VariableDroneConfig
    {
    private:
        double _power(const double weight) const
        {
            return beta * weight + gamma;
        }

    public:
        const double beta;
        const double gamma;

        DroneLinearConfig(
            const double capacity,
            const StatsType speed_type,
            const StatsType range_type,
            const double takeoff_speed,
            const double cruise_speed,
            const double landing_speed,
            const double altitude,
            const double battery,
            const double beta,
            const double gamma)
            : _VariableDroneConfig(
                  capacity,
                  speed_type,
                  range_type,
                  takeoff_speed,
                  cruise_speed,
                  landing_speed,
                  altitude,
                  battery),
              beta(beta),
              gamma(gamma)
        {
        }

        virtual ~DroneLinearConfig() = default;

        double takeoff_power(const double weight) const override
        {
            return _power(weight);
        }

        double landing_power(const double weight) const override
        {
            return _power(weight);
        }

        double cruise_power(const double weight) const override
        {
            return _power(weight);
        }
    };

    class DroneNonlinearConfig : public _VariableDroneConfig
    {
    private:
        static constexpr double W = 1.5;
        static constexpr double g = 9.8;

        double _vertical_power(const double weight, const double speed) const
        {
            double p = (W + weight) * g, half_speed = speed / 2;
            return k1 * p * (half_speed + utils::sqrt(utils::pow2(half_speed) + p / utils::pow2(k2))) + c2 * std::pow(p, 1.5);
        }

    public:
        const double k1;
        const double k2;
        const double c1;
        const double c2;
        const double c4;
        const double c5;

        DroneNonlinearConfig(
            const double capacity,
            const StatsType speed_type,
            const StatsType range_type,
            const double takeoff_speed,
            const double cruise_speed,
            const double landing_speed,
            const double altitude,
            const double battery,
            const double k1,
            const double k2,
            const double c1,
            const double c2,
            const double c4,
            const double c5)
            : _VariableDroneConfig(
                  capacity,
                  speed_type,
                  range_type,
                  takeoff_speed,
                  cruise_speed,
                  landing_speed,
                  altitude,
                  battery),
              k1(k1),
              k2(k2),
              c1(c1),
              c2(c2),
              c4(c4),
              c5(c5)
        {
        }

        virtual ~DroneNonlinearConfig() = default;

        double takeoff_power(const double weight) const override
        {
            return _vertical_power(weight, takeoff_speed);
        }

        double landing_power(const double weight) const override
        {
            return _vertical_power(weight, landing_speed);
        }

        double cruise_power(const double weight) const override
        {
            return (c1 + c2) * std::pow(utils::pow2((W + weight) * g - c5 * utils::pow2(cruise_speed * 0.984807753)) + utils::pow2(c4 * utils::pow2(cruise_speed)), 0.75) +
                   c4 * utils::pow3(cruise_speed);
        }
    };

    class DroneEnduranceConfig : public _BaseDroneConfig
    {
    public:
        const double fixed_time;
        // const double fixed_distance;
        const double drone_speed;

        DroneEnduranceConfig(
            const double capacity,
            const StatsType speed_type,
            const StatsType range_type,
            const double fixed_time,
            // const double fixed_distance,
            const double drone_speed)
            : _BaseDroneConfig(capacity, speed_type, range_type),
              fixed_time(fixed_time),
              // fixed_distance(fixed_distance),
              drone_speed(drone_speed)
        {
        }

        virtual ~DroneEnduranceConfig() = default;

        double takeoff_power(const double weight) const override
        {
            return 0;
        }

        double landing_power(const double weight) const override
        {
            return 0;
        }

        double cruise_power(const double weight) const override
        {
            return 0;
        }

        double takeoff_time() const override
        {
            return 0;
        }

        double landing_time() const override
        {
            return 0;
        }

        double cruise_time(const double distance) const override
        {
            return distance / drone_speed;
        }
    };
}
