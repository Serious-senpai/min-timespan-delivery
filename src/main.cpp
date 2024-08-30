#include <solutions.hpp>

int main()
{
    auto problem = d2d::Problem::get_instance();
    std::cerr << "tabu_size = " << problem->tabu_size << "\n";
    std::cerr << "verbose = " << problem->verbose << "\n";
    std::cerr << "trucks_count = " << problem->trucks_count << ", drones_count = " << problem->drones_count << "\n";
    std::cerr << "truck_time_limit = " << problem->truck_time_limit << "\n";
    std::cerr << "drone_time_limit = " << problem->drone_time_limit << "\n";
    std::cerr << "truck_unit_cost = " << problem->truck_unit_cost << "\n";
    std::cerr << "drone_unit_cost = " << problem->drone_unit_cost << "\n";
    std::cerr << "max_elite_set_size = " << problem->max_elite_set_size << ", reset_after = " << problem->reset_after << "\n";
    std::cerr << "hamming_distance_factor = " << problem->hamming_distance_factor << "\n";

    d2d::Logger<d2d::Solution> logger;
    auto ptr = d2d::Solution::tabu_search(logger);

    logger.finalize(ptr);

    return 0;
}
