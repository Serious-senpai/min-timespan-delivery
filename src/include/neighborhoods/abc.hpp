#pragma once

#include "../problem.hpp"

namespace d2d
{
    /**
     * @brief Base class for local search neighborhoods
     */
    template <typename ST>
    class Neighborhood
    {
    private:
        using _tabu_pair = std::pair<std::size_t, std::size_t>;

        /**
         * @brief Tabu list is usually small in size, therefore cache friendliness can outweigh
         * algorithm complexity
         */
        std::vector<_tabu_pair> _tabu_list;

    public:
        void add_to_tabu(const std::size_t &first, const std::size_t &second)
        {
            auto problem = Problem::get_instance();
            _tabu_pair p = std::minmax(first, second);
            auto tabu_iter = std::find(_tabu_list.begin(), _tabu_list.end(), p);
            if (tabu_iter == _tabu_list.end())
            {
                if (_tabu_list.size() == problem->tabu_size)
                {
                    _tabu_list.erase(_tabu_list.begin());
                }
                _tabu_list.push_back(p);
            }
            else
            {
                std::rotate(tabu_iter, tabu_iter + 1, _tabu_list.end());
            }
        }

        bool is_tabu(const std::size_t &first, const std::size_t &second) const
        {
            _tabu_pair p = std::minmax(first, second);
            return std::find(_tabu_list.begin(), _tabu_list.end(), p) != _tabu_list.end();
        }

        void clear()
        {
            _tabu_list.clear();
        }

        virtual std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> same_route(
            std::shared_ptr<ST> solution,
            const std::function<bool(std::shared_ptr<ST>)> &aspiration_criteria) = 0;

        virtual std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> multi_route(
            std::shared_ptr<ST> solution,
            const std::function<bool(std::shared_ptr<ST>)> &aspiration_criteria) = 0;

        virtual std::string performance_message() const = 0;

        /**
         * @brief Perform a local search to find the best solution in the neighborhood.
         *
         * @param solution A shared pointer to the current solution
         * @param aspiration_criteria The aspiration criteria of tabu search. This function should return `true`
         * if the solution satisfies the aspiration criteria, `false` otherwise
         * @return The best solution found that is not `solution`, or `nullptr` if the neighborhood is empty
         */
        std::shared_ptr<ST> move(
            std::shared_ptr<ST> solution,
            const std::function<bool(std::shared_ptr<ST>)> &aspiration_criteria)
        {
#ifdef DEBUG
            utils::PerformanceBenchmark _perf(performance_message());
#endif

            std::shared_ptr<ST> result;
            std::pair<std::size_t, std::size_t> tabu_pair;

            const auto update = [&result, &tabu_pair](const std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> &r)
            {
                if (r.first != nullptr && (result == nullptr || r.first->cost() < result->cost()))
                {
                    result = r.first;
                    tabu_pair = r.second;
                }
            };

            update(same_route(solution, aspiration_criteria));
            update(multi_route(solution, aspiration_criteria));

            if (result != nullptr)
            {
#ifdef DEBUG
                std::cerr << "Current solution:\n";
                std::cerr << "cost = " << solution->cost() << "\n";
                std::cerr << "truck_routes = " << solution->truck_routes << "\n";
                std::cerr << "drone_routes = " << solution->drone_routes << "\n";
                std::cerr << "New solution:\n";
                std::cerr << "cost = " << result->cost() << "\n";
                std::cerr << "truck_routes = " << result->truck_routes << "\n";
                std::cerr << "drone_routes = " << result->drone_routes << "\n";
                std::cerr << "Old tabu list = " << _tabu_list << "\n";
#endif

                this->add_to_tabu(tabu_pair.first, tabu_pair.second);

#ifdef DEBUG
                std::cerr << "New tabu list = " << _tabu_list << "\n";
#endif
            }

            return result;
        }
    };
}
