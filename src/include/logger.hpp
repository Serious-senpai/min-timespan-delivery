#pragma once

#include "parent.hpp"
#include "routes.hpp"

namespace d2d
{
    template <typename ST>
    class Logger
    {
    private:
        std::vector<std::shared_ptr<ST>> _history, _progress;
        std::vector<std::array<double, 4>> _coefficients;
        std::vector<std::vector<std::shared_ptr<ST>>> _elite_set;
        std::vector<std::pair<std::string, std::vector<std::size_t>>> _neighborhoods;

    public:
        std::size_t last_improved, iterations;
        std::chrono::milliseconds elapsed;

        void log(
            const std::shared_ptr<ST> history,
            const std::shared_ptr<ST> progress,
            const std::vector<std::shared_ptr<ST>> &elite_set,
            const std::pair<std::string, std::vector<std::size_t>> &neighborhood)
        {
            _history.push_back(history);
            _progress.push_back(progress);
            _coefficients.push_back(ST::penalty_coefficients());
            _elite_set.push_back(elite_set);
            _neighborhoods.push_back(neighborhood);
        }

        void print_solution(std::shared_ptr<ST> ptr)
        {
            if (ptr == nullptr)
            {
                std::cout << "-1\n";
                return;
            }

            std::cout << ptr->cost() << "\n";
            std::cout << ptr->working_time << "\n";
            std::cout << ptr->drone_energy_violation << "\n";
            std::cout << ptr->capacity_violation << "\n";
            std::cout << ptr->waiting_time_violation << "\n";
            std::cout << ptr->fixed_time_violation << "\n";

            std::cout << ptr->truck_routes << "\n";
            std::cout << ptr->drone_routes << "\n";

            std::cout << ptr->feasible << "\n";
        }

        void finalize(const std::shared_ptr<ST> ptr)
        {
            std::cout << std::fixed << std::setprecision(6);

            auto problem = d2d::Problem::get_instance();
            std::cout << iterations << "\n";

            std::cout << problem->tabu_size_factor << "\n";
            std::cout << problem->reset_after_factor << "\n";

            std::cout << problem->tabu_size << "\n";
            std::cout << problem->reset_after << "\n";
            std::cout << problem->max_elite_size << "\n";
            std::cout << problem->destroy_rate << "\n";

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

            std::string initialization_label;
            std::shared_ptr<ST> propagation = ptr;
            while (propagation != nullptr)
            {
                std::shared_ptr<ParentInfo<ST>> parent = propagation->parent();

                print_solution(propagation);
                std::cout << parent->label << "\n";

                initialization_label = parent->label;
                propagation = parent->ptr;
            }

            print_solution(nullptr);

            std::cout << _history.size() << "\n";
            for (auto &ptr : _history)
            {
                print_solution(ptr);
            }

            std::cout << _progress.size() << "\n";
            for (auto &ptr : _progress)
            {
                print_solution(ptr);
            }

            std::cout << _coefficients << std::endl;

            std::cout << _neighborhoods.size() << "\n";
            for (auto &neighborhood : _neighborhoods)
            {
                std::cout << neighborhood.first << "\n";
                std::cout << neighborhood.second << "\n";
            }

            std::cout << initialization_label << "\n";
            std::cout << last_improved << "\n";

            std::cout << _elite_set.size() << "\n";
            for (auto &elite_set : _elite_set)
            {
                std::vector<double> costs(elite_set.size());
                std::transform(
                    elite_set.begin(),
                    elite_set.end(),
                    costs.begin(),
                    [](const std::shared_ptr<ST> &ptr)
                    { return ptr->working_time; });
                std::cout << costs << "\n";
            }

            std::cout << elapsed.count() << "\n";
        }
    };
}
