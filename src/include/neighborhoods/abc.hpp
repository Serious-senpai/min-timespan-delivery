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
    public:
        /**
         * @brief Perform a local search to find the best solution in the neighborhood.
         *
         * @param solution A shared pointer to the current solution
         * @param aspiration_criteria The aspiration criteria of tabu search. This function should return `true`
         * if the solution satisfies the aspiration criteria, `false` otherwise
         * @return The best solution found that is not `solution`, or `nullptr` if the neighborhood is empty
         */
        virtual std::shared_ptr<ST> move(
            const std::shared_ptr<ST> &solution,
            const std::function<bool(const ST &)> &aspiration_criteria) = 0;
    };

    template <typename ST>
    class TabuPairNeighborhood : public Neighborhood<ST>
    {
    private:
        using tabu_pair = std::pair<std::size_t, std::size_t>;

        /**
         * @brief Tabu list is usually small in size, therefore cache friendliness can outweigh
         * algorithm complexity
         */
        std::vector<tabu_pair> tabu_list;

    protected:
        void add_to_tabu(const std::size_t &first, const std::size_t &second)
        {
            auto problem = Problem::get_instance();
            tabu_pair p = std::minmax(first, second);
            auto tabu_iter = std::find(tabu_list.begin(), tabu_list.end(), p);
            if (tabu_iter == tabu_list.end())
            {
                if (tabu_list.size() == problem->tabu_size)
                {
                    tabu_list.erase(tabu_list.begin());
                }
                tabu_list.push_back(p);
            }
            else
            {
                std::rotate(tabu_iter, tabu_iter + 1, tabu_list.end());
            }
        }

        bool is_tabu(const std::size_t &first, const std::size_t &second) const
        {
            tabu_pair p = std::minmax(first, second);
            return std::find(tabu_list.begin(), tabu_list.end(), p) != tabu_list.end();
        }
    };

    template <typename ST>
    class CommonRouteNeighborhood : public TabuPairNeighborhood<ST>
    {
    private:
        virtual std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> same_route(
            const std::shared_ptr<ST> &solution,
            const std::function<bool(const ST &)> &aspiration_criteria) = 0;

        virtual std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> multi_route(
            const std::shared_ptr<ST> &solution,
            const std::function<bool(const ST &)> &aspiration_criteria) = 0;

    public:
        virtual std::shared_ptr<ST> move(
            const std::shared_ptr<ST> &solution,
            const std::function<bool(const ST &)> &aspiration_criteria) override final
        {
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

            this->add_to_tabu(tabu_pair.first, tabu_pair.second);

            return result;
        }
    };
}
