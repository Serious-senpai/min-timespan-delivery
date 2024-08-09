#include <solutions.hpp>

void debug_display(const std::shared_ptr<d2d::Solution> &ptr)
{
    std::cout << ptr->truck_routes << std::endl;
    std::cout << ptr->drone_routes << std::endl;
    std::cout << ptr->cost() << std::endl;
}

void display(const std::shared_ptr<d2d::Solution> &ptr, const std::size_t &last_improved)
{
    std::cout << ptr->cost() << std::endl;
    std::cout << ptr->capacity_violation << std::endl;
    std::cout << ptr->drone_energy_violation << std::endl;
    std::cout << ptr->fixed_time_violation << std::endl;
    std::cout << ptr->fixed_distance_violation << std::endl;

#define PRINT_ROUTES(vehicle_routes)                     \
    {                                                    \
        for (auto &routes : ptr->vehicle_routes)         \
        {                                                \
            for (auto &route : routes)                   \
            {                                            \
                for (auto &customer : route.customers()) \
                {                                        \
                    std::cout << customer << " ";        \
                }                                        \
            }                                            \
            std::cout << std::endl;                      \
        }                                                \
    }

    PRINT_ROUTES(truck_routes);
    PRINT_ROUTES(drone_routes);

#undef PRINT_ROUTES

    std::cout << ptr->feasible << std::endl;
    std::cout << last_improved << std::endl;
}

int main()
{
    std::size_t last_improved;
    auto ptr = d2d::Solution::tabu_search(&last_improved);

#ifdef DEBUG
    debug_display(ptr);
#else
    display(ptr, last_improved);
#endif

    return 0;
}
