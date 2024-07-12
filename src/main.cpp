#include <solutions.hpp>

int main()
{
    auto ptr = d2d::Solution::initial();
    std::cout << ptr->truck_routes << std::endl;
    std::cout << ptr->drone_routes << std::endl;
    std::cout << ptr->working_time << std::endl;

    return 0;
}
