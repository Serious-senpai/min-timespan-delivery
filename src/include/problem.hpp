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
    };

    class Problem
    {
    private:
        static Problem *_instance;

        Problem(
            const std::size_t &tabu_size,
            const bool verbose,
            const std::size_t &trucks_count,
            const std::size_t &drones_count,
            const std::vector<Customer> &customers,
            const std::vector<std::vector<double>> &distances,
            const double &total_demand,
            const TruckConfig *const truck,
            const _BaseDroneConfig *const drone,
            const DroneLinearConfig *const linear,
            const DroneNonlinearConfig *const nonlinear,
            const DroneEnduranceConfig *const endurance,
            const std::size_t &max_elite_set_size,
            const std::size_t &reset_after,
            const double &hamming_distance_factor)
            : tabu_size(tabu_size),
              verbose(verbose),
              trucks_count(trucks_count),
              drones_count(drones_count),
              customers(customers),
              distances(distances),
              total_demand(total_demand),
              truck(truck),
              drone(drone),
              linear(linear),
              nonlinear(nonlinear),
              endurance(endurance),
              max_elite_set_size(max_elite_set_size),
              reset_after(reset_after),
              hamming_distance_factor(hamming_distance_factor) {}

        ~Problem()
        {
            delete truck;
            delete drone;
        }

    public:
        const std::size_t tabu_size;
        const bool verbose;
        const std::size_t trucks_count, drones_count;
        const std::vector<Customer> customers;
        const std::vector<std::vector<double>> distances;
        const double maximum_waiting_time = 3600; // hard-coded value
        const double total_demand;
        const TruckConfig *const truck;
        const _BaseDroneConfig *const drone;
        const DroneLinearConfig *const linear;
        const DroneNonlinearConfig *const nonlinear;
        const DroneEnduranceConfig *const endurance;

        const std::size_t max_elite_set_size, reset_after;
        const double hamming_distance_factor;

        static Problem *get_instance();
    };

    Problem *Problem::_instance = nullptr;
    Problem *Problem::get_instance()
    {
        if (_instance == nullptr)
        {
            std::size_t customers_count, trucks_count, drones_count;
            std::cin >> customers_count >> trucks_count >> drones_count;

            std::vector<double> x(customers_count + 1);
            for (std::size_t i = 0; i < customers_count + 1; i++)
            {
                std::cin >> x[i];
            }

            std::vector<double> y(customers_count + 1);
            for (std::size_t i = 0; i < customers_count + 1; i++)
            {
                std::cin >> y[i];
            }

            std::vector<double> demands(customers_count + 1);
            for (std::size_t i = 0; i < customers_count + 1; i++)
            {
                std::cin >> demands[i];
            }

            std::vector<bool> dronable;
            for (std::size_t i = 0; i < customers_count + 1; i++)
            {
                bool b;
                std::cin >> b;
                dronable.push_back(b);
            }

            std::vector<double> truck_service_time(customers_count + 1);
            for (std::size_t i = 0; i < customers_count + 1; i++)
            {
                std::cin >> truck_service_time[i];
            }

            std::vector<double> drone_service_time(customers_count + 1);
            for (std::size_t i = 0; i < customers_count + 1; i++)
            {
                std::cin >> drone_service_time[i];
            }

            std::vector<Customer> customers;
            for (std::size_t i = 0; i < customers_count + 1; i++)
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

            std::size_t tabu_size;
            bool verbose;
            std::cin >> tabu_size >> verbose;

            double truck_maximum_velocity, truck_capacity;
            std::cin >> truck_maximum_velocity >> truck_capacity;

            std::size_t truck_coefficients_count;
            std::cin >> truck_coefficients_count;
            std::vector<double> truck_coefficients(truck_coefficients_count);
            for (std::size_t i = 0; i < truck_coefficients_count; i++)
            {
                std::cin >> truck_coefficients[i];
            }

            TruckConfig *truck = new TruckConfig(
                truck_maximum_velocity,
                truck_coefficients,
                truck_capacity);

            std::string drone_class;
            std::cin >> drone_class;

            double capacity;
            std::string _speed_type;
            std::string _range_type;
            std::cin >> capacity >> _speed_type >> _range_type;

            StatsType speed_type = _speed_type == "low" ? StatsType::low : StatsType::high,
                      range_type = _range_type == "low" ? StatsType::low : StatsType::high;

            _BaseDroneConfig *drone = nullptr;
            if (drone_class == "DroneLinearConfig")
            {
                double takeoff_speed, cruise_speed, landing_speed, altitude, battery, beta, gamma;
                std::cin >> takeoff_speed >> cruise_speed >> landing_speed >> altitude >> battery >> beta >> gamma;
                drone = new DroneLinearConfig(
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
            else if (drone_class == "DroneNonlinearConfig")
            {
                double takeoff_speed, cruise_speed, landing_speed, altitude, battery, k1, k2, c1, c2, c4, c5;
                std::cin >> takeoff_speed >> cruise_speed >> landing_speed >> altitude >> battery >> k1 >> k2 >> c1 >> c2 >> c4 >> c5;
                drone = new DroneNonlinearConfig(
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
            else if (drone_class == "DroneEnduranceConfig")
            {
                double fixed_time, fixed_distance, drone_speed;
                std::cin >> fixed_time >> fixed_distance >> drone_speed;
                drone = new DroneEnduranceConfig(
                    capacity,
                    speed_type,
                    range_type,
                    fixed_time,
                    fixed_distance,
                    drone_speed);
            }
            else
            {
                throw std::runtime_error(utils::format("Unknown drone energy model \"%s\"", drone_class.c_str()));
            }

            std::size_t max_elite_set_size, reset_after;
            double hamming_distance_factor;
            std::cin >> max_elite_set_size >> reset_after >> hamming_distance_factor;

            _instance = new Problem(
                tabu_size,
                verbose,
                trucks_count,
                drones_count,
                customers,
                distances,
                std::accumulate(
                    customers.begin(), customers.end(), 0.0,
                    [](const double &sum, const Customer &customer)
                    { return sum + customer.demand; }),
                truck,
                drone,
                dynamic_cast<DroneLinearConfig *>(drone),
                dynamic_cast<DroneNonlinearConfig *>(drone),
                dynamic_cast<DroneEnduranceConfig *>(drone),
                max_elite_set_size,
                reset_after,
                hamming_distance_factor);
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
