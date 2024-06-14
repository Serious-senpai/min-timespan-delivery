#pragma once

#include "config.hpp"
#include "format.hpp"

namespace d2d
{
    class Customer
    {
    public:
        const double x, y;
        const double demand;
        const bool dronable;
        const double truck_service_time;
        const double drone_service_time;

        Customer(
            const double x,
            const double y,
            const double demand,
            const bool dronable,
            const double truck_service_time,
            const double drone_service_time)
            : x(x),
              y(y),
              demand(demand),
              dronable(dronable),
              truck_service_time(truck_service_time),
              drone_service_time(drone_service_time) {}

        static Customer depot();
    };

    Customer Customer::depot()
    {
        return Customer(0, 0, 0, true, 0, 0);
    }

    class Problem
    {
    private:
        static Problem *_instance;

        Problem(
            const std::size_t &iterations,
            const std::size_t &tabu_size,
            const std::size_t &customers_count,
            const std::size_t &trucks_count,
            const std::size_t &drones_count,
            const std::vector<Customer> &customers,
            const std::vector<std::vector<double>> &distances,
            _BaseDroneConfig *const config)
            : iterations(iterations),
              tabu_size(tabu_size),
              customers_count(customers_count),
              trucks_count(trucks_count),
              drones_count(drones_count),
              customers(customers),
              distances(distances),
              config(config) {}

        ~Problem()
        {
            delete config;
        }

    public:
        const std::size_t iterations, tabu_size;
        const std::size_t customers_count, trucks_count, drones_count;
        const std::vector<Customer> customers;
        const std::vector<std::vector<double>> distances;
        _BaseDroneConfig *const config;

        static Problem *get_instance();
    };

    Problem *Problem::_instance = nullptr;
    Problem *Problem::get_instance()
    {
        if (_instance == nullptr)
        {
            std::size_t customers_count, trucks_count, drones_count;
            std::cin >> customers_count >> trucks_count >> drones_count;

            std::vector<double> x(customers_count);
            for (std::size_t i = 0; i < customers_count; i++)
            {
                std::cin >> x[i];
            }

            std::vector<double> y(customers_count);
            for (std::size_t i = 0; i < customers_count; i++)
            {
                std::cin >> y[i];
            }

            std::vector<double> demands(customers_count);
            for (std::size_t i = 0; i < customers_count; i++)
            {
                std::cin >> demands[i];
            }

            std::vector<bool> dronable;
            for (std::size_t i = 0; i < customers_count; i++)
            {
                bool b;
                std::cin >> b;
                dronable.push_back(b);
            }

            std::vector<double> truck_service_time(customers_count);
            for (std::size_t i = 0; i < customers_count; i++)
            {
                std::cin >> truck_service_time[i];
            }

            std::vector<double> drone_service_time(customers_count);
            for (std::size_t i = 0; i < customers_count; i++)
            {
                std::cin >> drone_service_time[i];
            }

            std::vector<Customer> customers;
            customers.push_back(Customer::depot());
            for (std::size_t i = 0; i < customers_count; i++)
            {
                customers.emplace_back(x[i], y[i], demands[i], dronable[i], truck_service_time[i], drone_service_time[i]);
            }

            std::vector<std::vector<double>> distances(customers.size(), std::vector<double>(customers.size()));
            for (std::size_t i = 0; i < customers.size(); i++)
            {
                for (std::size_t j = i + 1; j < customers.size(); j++)
                {
                    distances[i][j] = distances[j][i] = utils::distance(
                        customers[i].x - customers[j].x,
                        customers[i].y - customers[j].y);
                }
            }

            std::size_t iterations, tabu_size;
            std::cin >> iterations >> tabu_size;

            std::string config_class;
            std::cin >> config_class;

            double capacity;
            std::string _speed_type;
            std::string _range_type;
            std::cin >> capacity >> _speed_type >> _range_type;

            StatsType speed_type = _speed_type == "low" ? StatsType::low : StatsType::high,
                      range_type = _range_type == "low" ? StatsType::low : StatsType::high;

            _BaseDroneConfig *config = nullptr;
            if (config_class == "DroneLinearConfig")
            {
                double takeoff_speed, cruise_speed, landing_speed, altitude, battery, beta, gamma;
                std::cin >> takeoff_speed >> cruise_speed >> landing_speed >> altitude >> battery >> beta >> gamma;
                config = new DroneLinearConfig(
                    capacity,
                    speed_type,
                    range_type,
                    takeoff_speed,
                    cruise_speed,
                    landing_speed,
                    altitude,
                    battery,
                    beta,
                    gamma);
            }
            else if (config_class == "DroneNonlinearConfig")
            {
                double takeoff_speed, cruise_speed, landing_speed, altitude, battery, k1, k2, c1, c2, c4, c5;
                std::cin >> takeoff_speed >> cruise_speed >> landing_speed >> altitude >> battery >> k1 >> k2 >> c1 >> c2 >> c4 >> c5;
                config = new DroneNonlinearConfig(
                    capacity,
                    speed_type,
                    range_type,
                    takeoff_speed,
                    cruise_speed,
                    landing_speed,
                    altitude,
                    battery,
                    k1,
                    k2,
                    c1,
                    c2,
                    c4,
                    c5);
            }
            else if (config_class == "DroneEnduranceConfig")
            {
                double fixed_time, fixed_distance, drone_speed;
                std::cin >> fixed_time >> fixed_distance >> drone_speed;
                config = new DroneEnduranceConfig(
                    capacity,
                    speed_type,
                    range_type,
                    fixed_time,
                    fixed_distance,
                    drone_speed);
            }
            else
            {
                throw std::runtime_error(utils::format("Unknown drone energy model \"%s\"", config_class.c_str()));
            }

            _instance = new Problem(
                iterations,
                tabu_size,
                customers_count,
                trucks_count,
                drones_count,
                customers,
                distances,
                config);
        }

        return _instance;
    }
}

namespace std
{
    ostream &operator<<(ostream &stream, const d2d::Customer &customer)
    {
        stream << "Customer(x=" << customer.x << ", y=" << customer.y << ", demand=" << customer.demand << ", dronable=" << customer.dronable << ")";
        return stream;
    }
}
