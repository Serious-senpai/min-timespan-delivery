#include <solutions.hpp>

int main()
{
    auto problem = d2d::Problem::get_instance();
    std::cout << problem->drones_count << std::endl;
    std::cout << problem->endurance->drone_speed << std::endl;

    d2d::Logger<d2d::Solution> logger;
    utils::PerformanceBenchmark benchmark("Elapsed");
    auto ptr = d2d::Solution::tabu_search(logger);

    std::cerr << "\e[31mResult = " << ptr->cost() << "\e[0m" << std::endl;

    logger.elapsed = benchmark.elapsed<std::chrono::milliseconds>();
    logger.finalize(ptr);

    return 0;
}
