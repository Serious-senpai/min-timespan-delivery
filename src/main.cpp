#include <problem.hpp>

int main()
{
    auto problem = d2d::Problem::get_instance();
    auto ptr = dynamic_cast<d2d::DroneLinearConfig *>(problem->config);
    if (ptr != NULL)
    {
        std::cout << ptr->beta << std::endl;
        std::cout << ptr->gamma << std::endl;
    }

    return 0;
}
