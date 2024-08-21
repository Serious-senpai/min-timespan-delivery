#include <solutions.hpp>

void print_solution(std::shared_ptr<d2d::Solution> ptr)
{
    std::cout << ptr->cost() << std::endl;
    std::cout << ptr->working_time << std::endl;
    std::cout << ptr->capacity_violation << std::endl;
    std::cout << ptr->drone_energy_violation << std::endl;
    std::cout << ptr->waiting_time_violation << std::endl;
    std::cout << ptr->fixed_time_violation << std::endl;
    std::cout << ptr->fixed_distance_violation << std::endl;

    std::cout << ptr->truck_routes << std::endl;
    std::cout << ptr->drone_routes << std::endl;

    std::cout << ptr->feasible << std::endl;
}

void display(std::shared_ptr<d2d::Solution> ptr, const std::size_t &last_improved)
{
    auto problem = d2d::Problem::get_instance();
    std::cout << problem->iterations << std::endl;
    std::cout << problem->tabu_size << std::endl;

    if (problem->linear != nullptr)
    {
        std::cout << "linear" << std::endl;
    }
    else if (problem->nonlinear != nullptr)
    {
        std::cout << "nonlinear" << std::endl;
    }
    else if (problem->endurance != nullptr)
    {
        std::cout << "endurance" << std::endl;
    }
    else
    {
        throw std::runtime_error("No drone configuration was found. This should never happen.");
    }

    std::cout << (problem->drone->speed_type == d2d::StatsType::low ? "low" : "high") << std::endl;
    std::cout << (problem->drone->range_type == d2d::StatsType::low ? "low" : "high") << std::endl;

    print_solution(ptr);

    auto parent = ptr->parent();
    while (parent != nullptr)
    {
        print_solution(parent->ptr);
        std::cout << parent->label << std::endl;
        parent = parent->ptr->parent();
    }
    std::cout << -1 << std::endl; // cost = -1. Signal the end of propagation chain.

    std::cout << last_improved << std::endl;
}

int main()
{
    std::size_t last_improved;
    auto ptr = d2d::Solution::tabu_search(&last_improved);

    display(ptr, last_improved);

    return 0;
}
