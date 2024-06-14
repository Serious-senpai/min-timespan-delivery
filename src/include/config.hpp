#pragma once

#include "standard.hpp"
#include "utils.hpp"

namespace d2d
{
    class TruckConfig
    {
    public:
        const double maximum_velocity;
        const double capacity;
        const std::vector<double> coefficients;

        TruckConfig(
            const double maximum_velocity,
            const double capacity,
            const std::vector<double> coefficients)
            : maximum_velocity(maximum_velocity),
              capacity(capacity),
              coefficients(coefficients)
        {
        }

        double speed(const std::size_t &index) const
        {
            return maximum_velocity * coefficients[index % coefficients.size()];
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

        double _vertical_speed(const double weight, const double speed) const
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
            return _vertical_speed(weight, takeoff_speed);
        }

        double landing_power(const double weight) const override
        {
            return _vertical_speed(weight, landing_speed);
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
        const double fixed_distance;
        const double drone_speed;

        DroneEnduranceConfig(
            const double capacity,
            const StatsType speed_type,
            const StatsType range_type,
            const double fixed_time,
            const double fixed_distance,
            const double drone_speed)
            : _BaseDroneConfig(capacity, speed_type, range_type),
              fixed_time(fixed_time),
              fixed_distance(fixed_distance),
              drone_speed(drone_speed)
        {
        }

        virtual ~DroneEnduranceConfig() = default;

        double takeoff_power(const double weight) const override
        {
            return std::numeric_limits<double>::infinity();
        }

        double landing_power(const double weight) const override
        {
            return std::numeric_limits<double>::infinity();
        }

        double cruise_power(const double weight) const override
        {
            return drone_speed;
        }
    };
}
