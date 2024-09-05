#pragma once

#include "../parent.hpp"
#include "../problem.hpp"
#include "../routes.hpp"

namespace d2d
{
    /**
     * @brief Base class for local search neighborhoods
     */
    template <typename ST>
    class BaseNeighborhood
    {
    public:
        virtual std::pair<std::shared_ptr<ST>, std::vector<std::size_t>> intra_route(
            const std::shared_ptr<ST> solution,
            const std::function<bool(std::shared_ptr<ST>)> &aspiration_criteria) = 0;

        virtual std::pair<std::shared_ptr<ST>, std::vector<std::size_t>> inter_route(
            const std::shared_ptr<ST> solution,
            const std::function<bool(std::shared_ptr<ST>)> &aspiration_criteria) = 0;

        virtual std::string label() const = 0;

        virtual std::shared_ptr<ST> construct(
            const std::shared_ptr<ParentInfo<ST>> parent,
            const std::vector<std::vector<TruckRoute>> &truck_routes,
            const std::vector<std::vector<DroneRoute>> &drone_routes) const final
        {
            return std::make_shared<ST>(truck_routes, drone_routes, parent);
        }

        virtual std::shared_ptr<ParentInfo<ST>> parent_ptr(const std::shared_ptr<ST> solution) const final
        {
            return std::make_shared<ParentInfo<ST>>(solution, label());
        }
    };

    template <typename ST, bool _EnableTabuList>
    class Neighborhood : public BaseNeighborhood<ST>
    {
    };

    template <typename ST>
    class Neighborhood<ST, true> : public BaseNeighborhood<ST>
    {
    private:
        /**
         * @brief Tabu list is usually small in size, therefore cache friendliness can outweigh
         * algorithm complexity
         */
        std::vector<std::vector<std::size_t>> _tabu_list;

        static const std::vector<std::size_t> _empty_tabu_id;

    public:
        const std::vector<std::size_t> &last_tabu() const
        {
            if (_tabu_list.empty())
            {
                return _empty_tabu_id;
            }

            return _tabu_list.back();
        }

        template <typename... Args>
        void add_to_tabu(const std::size_t &tabu_id, const Args &...tabu_ids)
        {
            std::vector<std::size_t> t = {tabu_id, tabu_ids...};
            add_to_tabu(t);
        }

        void add_to_tabu(std::vector<std::size_t> &tabu_id)
        {
            auto problem = Problem::get_instance();
            std::sort(tabu_id.begin(), tabu_id.end());

            auto tabu_iter = std::find(_tabu_list.begin(), _tabu_list.end(), tabu_id);
            if (tabu_iter == _tabu_list.end())
            {
                if (_tabu_list.size() == problem->tabu_size)
                {
                    _tabu_list.erase(_tabu_list.begin());
                }
                _tabu_list.push_back(tabu_id);
            }
            else
            {
                std::rotate(tabu_iter, tabu_iter + 1, _tabu_list.end());
            }
        }

        template <typename... Args>
        bool is_tabu(const std::size_t &tabu_id, const Args &...tabu_ids) const
        {
            std::vector<std::size_t> t = {tabu_id, tabu_ids...};
            return is_tabu(t);
        }

        bool is_tabu(std::vector<std::size_t> &tabu_id) const
        {
            std::sort(tabu_id.begin(), tabu_id.end());
            return std::find(_tabu_list.begin(), _tabu_list.end(), tabu_id) != _tabu_list.end();
        }

        void clear()
        {
            _tabu_list.clear();
        }

        /**
         * @brief Perform a local search to find the best solution in the neighborhood.
         *
         * @param solution A shared pointer to the current solution
         * @param aspiration_criteria The aspiration criteria of tabu search. This function should return `true`
         * if the solution satisfies the aspiration criteria, `false` otherwise
         * @return The best solution found that is not `solution`, or `nullptr` if the neighborhood is empty
         */
        std::shared_ptr<ST> move(
            const std::shared_ptr<ST> solution,
            const std::function<bool(const std::shared_ptr<ST>)> &aspiration_criteria)
        {
#ifdef DEBUG
            utils::PerformanceBenchmark _perf(this->label());
#endif

            std::shared_ptr<ST> result;
            std::vector<std::size_t> tabu;

            const auto update = [&result, &tabu](const std::pair<std::shared_ptr<ST>, std::vector<std::size_t>> &r)
            {
                if (r.first != nullptr && (result == nullptr || r.first->cost() < result->cost()))
                {
                    result = r.first;
                    tabu = r.second;
                }
            };

            update(this->intra_route(solution, aspiration_criteria));
            update(this->inter_route(solution, aspiration_criteria));

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

                this->add_to_tabu(tabu);

#ifdef DEBUG
                std::cerr << "New tabu list = " << _tabu_list << "\n";
#endif
            }

            return result;
        }
    };

    template <typename ST>
    const std::vector<std::size_t> Neighborhood<ST, true>::_empty_tabu_id;
}
