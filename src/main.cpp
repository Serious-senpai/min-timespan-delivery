#include <solutions.hpp>

int main()
{
    d2d::Logger<d2d::Solution> logger;
    utils::PerformanceBenchmark benchmark("Elapsed");
    auto ptr = d2d::Solution::tabu_search(logger);

    std::cerr << "\e[31mResult = " << ptr->cost() << "\e[0m" << std::endl;

    logger.elapsed = benchmark.elapsed<std::chrono::milliseconds>();
    logger.finalize(ptr);

    return 0;
}
