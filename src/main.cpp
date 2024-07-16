#include <solutions.hpp>

int main()
{
    auto ptr = d2d::Solution::tabu_search();
    std::cout << ptr->truck_routes << std::endl;
    std::cout << ptr->drone_routes << std::endl;
    std::cout << ptr->cost() << std::endl;

    return 0;
}
