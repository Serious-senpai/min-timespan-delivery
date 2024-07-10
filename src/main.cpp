#include <problem.hpp>
#include <routes.hpp>

#include <iomanip>

int main()
{
    std::cout << std::fixed << std::setprecision(3);
    auto route = d2d::DroneRoute({0, 1, 2, 3, 0});
    std::cout << route.distance << std::endl;
    std::cout << route.weight << std::endl;
    std::cout << route.time_segments << std::endl;
    std::cout << route.energy_consumption << std::endl;

    auto problem = d2d::Problem::get_instance();
    std::cout << "Distance matrix:\n";
    for (auto &row : problem->distances)
    {
        std::cout << row << std::endl;
    }
    std::cout << problem->distances[0][1] << " " << problem->distances[1][2] << " " << problem->distances[2][3] << " " << problem->distances[3][0] << std::endl;

    std::cout << "Customers:\n";
    for (std::size_t i = 0; i < problem->customers.size(); i++)
    {
        std::cout << i << " " << problem->customers[i] << std::endl;
    }

    std::cout << "Truck speed during first 5 hours: ";
    for (std::size_t i = 0; i < 5; i++)
    {
        std::cout << problem->truck->speed(i) << " ";
    }
    std::cout << std::endl;

    std::cout << "Drone takeoff time: " << problem->drone->takeoff_time() << std::endl;
    std::cout << "Drone landing time: " << problem->drone->landing_time() << std::endl;
    std::cout << "Drone cruising speed: " << problem->linear->cruise_speed << std::endl;

    std::cout << "Drone takeoff power:";
    double w = 0;
    for (std::size_t i = 0; i < 4; i++)
    {
        w += problem->customers[i].demand;
        std::cout << " " << problem->drone->takeoff_power(w) << " ";
    }
    std::cout << std::endl;

    std::cout << "Drone landing power:";
    w = 0;
    for (std::size_t i = 0; i < 4; i++)
    {
        w += problem->customers[i].demand;
        std::cout << " " << problem->drone->landing_power(w) << " ";
    }
    std::cout << std::endl;

    std::cout << "Drone cruising power:";
    w = 0;
    for (std::size_t i = 0; i < 4; i++)
    {
        w += problem->customers[i].demand;
        std::cout << " " << problem->drone->cruise_power(w) << " ";
    }
    std::cout << std::endl;

    return 0;
}
