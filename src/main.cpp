#include <solutions.hpp>

void print_solution(std::shared_ptr<d2d::Solution> ptr)
{
    std::cout << ptr->cost() << "\n";
    std::cout << ptr->travel_cost << "\n";
    std::cout << ptr->capacity_violation << "\n";
    std::cout << ptr->drone_energy_violation << "\n";
    std::cout << ptr->working_time_violation << "\n";
    std::cout << ptr->fixed_time_violation << "\n";
    std::cout << ptr->fixed_distance_violation << "\n";

    std::cout << ptr->truck_routes << "\n";
    std::cout << ptr->drone_routes << "\n";

    std::cout << ptr->feasible << "\n";
}

void display(
    std::shared_ptr<d2d::Solution> ptr,
    const std::size_t &last_improved,
    const std::vector<std::shared_ptr<d2d::Solution>> &history,
    const std::vector<std::shared_ptr<d2d::Solution>> &progress,
    const std::vector<std::array<double, 5>> &coefficients)
{
    auto problem = d2d::Problem::get_instance();
    std::cout << problem->iterations << "\n";
    std::cout << problem->tabu_size << "\n";

    if (problem->linear != nullptr)
    {
        std::cout << "linear\n";
    }
    else if (problem->nonlinear != nullptr)
    {
        std::cout << "nonlinear\n";
    }
    else if (problem->endurance != nullptr)
    {
        std::cout << "endurance\n";
    }
    else
    {
        throw std::runtime_error("No drone configuration was found. This should never happen.");
    }

    std::cout << (problem->drone->speed_type == d2d::StatsType::low ? "low" : "high") << "\n";
    std::cout << (problem->drone->range_type == d2d::StatsType::low ? "low" : "high") << "\n";

    print_solution(ptr);

    auto parent = ptr->parent();
    while (parent != nullptr)
    {
        print_solution(parent->ptr);
        std::cout << parent->label << "\n";
        parent = parent->ptr->parent();
    }
    std::cout << "-1\n"; // cost = -1. Signal the end of propagation chain.

    // Remove duplicate history logging
    std::shared_ptr<d2d::Solution> last;
    for (std::size_t iteration = 0; iteration < history.size(); iteration++)
    {
        if (history[iteration] != last)
        {
            last = history[iteration];
            print_solution(last);
            std::cout << iteration << "\n";
            std::cout << coefficients[iteration] << "\n";
        }
    }
    std::cout << "-1\n"; // cost = -1. Signal the end of history chain.

    for (std::size_t iteration = 0; iteration < history.size(); iteration++)
    {
        print_solution(progress[iteration]);
        std::cout << coefficients[iteration] << "\n";
    }
    std::cout << "-1\n"; // cost = -1. Signal the end of progress chain.

    std::cout << last_improved << "\n";
}

int main()
{
    std::size_t last_improved;
    std::vector<std::shared_ptr<d2d::Solution>> history, progress;
    std::vector<std::array<double, 5>> coefficients;
    auto ptr = d2d::Solution::tabu_search(&last_improved, &history, &progress, &coefficients);

    display(ptr, last_improved, history, progress, coefficients);

    return 0;
}
