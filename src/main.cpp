#include <solutions.hpp>

int main()
{
    d2d::Logger<d2d::Solution> logger;
    utils::PerformanceBenchmark benchmark("Elapsed");

    auto problem = d2d::Problem::get_instance();
    std::shared_ptr<d2d::Solution> ptr;
    if (problem->evaluate.has_value())
    {
        auto &[truck_routes, drone_routes] = problem->evaluate.value();

        std::vector<std::vector<d2d::TruckRoute>> _truck_routes(problem->trucks_count);
        for (std::size_t truck = 0; truck < problem->trucks_count; truck++)
        {
            for (auto &route : truck_routes[truck])
            {
                _truck_routes[truck].emplace_back(route);
            }
        }

        std::vector<std::vector<d2d::DroneRoute>> _drone_routes(problem->drones_count);
        for (std::size_t drone = 0; drone < problem->drones_count; drone++)
        {
            for (auto &route : drone_routes[drone])
            {
                _drone_routes[drone].emplace_back(route);
            }
        }

        ptr = std::make_shared<d2d::Solution>(_truck_routes, _drone_routes, std::make_shared<d2d::ParentInfo<d2d::Solution>>(nullptr, "evaluate"));
    }
    else
    {
        ptr = d2d::Solution::tabu_search(logger);
    }

    std::cerr << "\e[31mResult = " << ptr->cost() << "\e[0m" << std::endl;

    logger.elapsed = benchmark.elapsed<std::chrono::milliseconds>();
    logger.finalize(ptr);

    return 0;
}
