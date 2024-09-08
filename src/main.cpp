#include <solutions.hpp>

int main()
{
    auto problem = d2d::Problem::get_instance();
    std::cerr << "tabu_size = " << problem->tabu_size << "\n";
    std::cerr << "verbose = " << problem->verbose << "\n";
    std::cerr << "trucks_count = " << problem->trucks_count << ", drones_count = " << problem->drones_count << "\n";
    std::cerr << "maximum_waiting_time = " << problem->maximum_waiting_time << "\n";
    std::cerr << "max_elite_set_size = " << problem->max_elite_set_size << ", reset_after = " << problem->reset_after << "\n";
    std::cerr << "hamming_distance_factor = " << problem->hamming_distance_factor << "\n";

    d2d::Logger<d2d::Solution> logger;
    utils::PerformanceBenchmark benchmark("Elapsed");
    auto ptr = d2d::Solution::tabu_search(logger);

    std::cerr << "\e[31mResult = " << ptr->cost() << "\e[0m" << std::endl;

    logger.elapsed = benchmark.elapsed<std::chrono::milliseconds>();
    logger.finalize(ptr);

    return 0;
}
