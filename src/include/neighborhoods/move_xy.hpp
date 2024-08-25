#pragma once

#include "abc.hpp"

namespace d2d
{
    template <typename ST, std::size_t X, std::size_t Y>
    class _BaseMoveXY : public Neighborhood<ST, true>
    {
        static_assert(X >= Y && X != 0);

    public:
        std::string label() const override
        {
            return utils::format("Move (%d, %d)", X, Y);
        }

        std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> inter_route(
            std::shared_ptr<ST> solution,
            const std::function<bool(std::shared_ptr<ST>)> &aspiration_criteria) override
        {
            auto problem = Problem::get_instance();
            auto parent = this->parent_ptr(solution);
            std::shared_ptr<ST> result;
            std::pair<std::size_t, std::size_t> tabu_pair;

            std::vector<std::vector<TruckRoute>> truck_routes(solution->truck_routes);
            std::vector<std::vector<DroneRoute>> drone_routes(solution->drone_routes);

            for (std::size_t vehicle_i = 0; vehicle_i < problem->trucks_count + problem->drones_count; vehicle_i++)
            {
                std::size_t offset_j = 0;
                if constexpr (X == Y)
                {
                    offset_j = vehicle_i;
                }

                for (std::size_t vehicle_j = offset_j; vehicle_j < problem->trucks_count + problem->drones_count; vehicle_j++)
                {
#define MODIFY_ROUTES(vehicle_routes_i, vehicle_routes_j)                                                                              \
    {                                                                                                                                  \
        std::size_t _vehicle_i = vehicle_i, _vehicle_j = vehicle_j;                                                                    \
        if constexpr (std::is_same_v<std::remove_cvref_t<decltype(solution->vehicle_routes_i)>, std::vector<std::vector<DroneRoute>>>) \
        {                                                                                                                              \
            _vehicle_i -= problem->trucks_count;                                                                                       \
        }                                                                                                                              \
                                                                                                                                       \
        if constexpr (std::is_same_v<std::remove_cvref_t<decltype(solution->vehicle_routes_j)>, std::vector<std::vector<DroneRoute>>>) \
        {                                                                                                                              \
            _vehicle_j -= problem->trucks_count;                                                                                       \
        }                                                                                                                              \
                                                                                                                                       \
        for (std::size_t route_i = 0; route_i < solution->vehicle_routes_i[_vehicle_i].size(); route_i++)                              \
        {                                                                                                                              \
            for (std::size_t route_j = 0; route_j < solution->vehicle_routes_j[_vehicle_j].size(); route_j++)                          \
            {                                                                                                                          \
                const std::vector<std::size_t> &customers_i = solution->vehicle_routes_i[_vehicle_i][route_i].customers();             \
                const std::vector<std::size_t> &customers_j = solution->vehicle_routes_j[_vehicle_j][route_j].customers();             \
                for (std::size_t i = 1; i + X < customers_i.size(); i++)                                                               \
                {                                                                                                                      \
                    for (std::size_t j = 1; j + Y < customers_j.size(); j++)                                                           \
                    {                                                                                                                  \
                        using VehicleRoute_i = std::remove_cvref_t<decltype(vehicle_routes_i[_vehicle_i][route_i])>;                   \
                        using VehicleRoute_j = std::remove_cvref_t<decltype(vehicle_routes_j[_vehicle_j][route_j])>;                   \
                        if constexpr (std::is_same_v<VehicleRoute_i, VehicleRoute_j>)                                                  \
                        {                                                                                                              \
                            if (_vehicle_i == _vehicle_j && route_i == route_j) /* same route */                                       \
                            {                                                                                                          \
                                continue;                                                                                              \
                            }                                                                                                          \
                        }                                                                                                              \
                                                                                                                                       \
                        std::vector<std::size_t> ri(customers_i.begin(), customers_i.begin() + i),                                     \
                            rj(customers_j.begin(), customers_j.begin() + j);                                                          \
                                                                                                                                       \
                        ri.insert(ri.end(), customers_j.begin() + j, customers_j.begin() + (j + Y));                                   \
                        rj.insert(rj.end(), customers_i.begin() + i, customers_i.begin() + (i + X));                                   \
                                                                                                                                       \
                        ri.insert(ri.end(), customers_i.begin() + (i + X), customers_i.end());                                         \
                        rj.insert(rj.end(), customers_j.begin() + (j + Y), customers_j.end());                                         \
                                                                                                                                       \
                        if constexpr (std::is_same_v<VehicleRoute_i, DroneRoute> && std::is_same_v<VehicleRoute_j, TruckRoute>)        \
                        {                                                                                                              \
                            if (std::any_of(                                                                                           \
                                    customers_j.begin() + j, customers_j.begin() + (j + Y),                                            \
                                    [&problem](const std::size_t &c) { return !problem->customers[c].dronable; }))                     \
                            {                                                                                                          \
                                continue;                                                                                              \
                            }                                                                                                          \
                        }                                                                                                              \
                                                                                                                                       \
                        if constexpr (std::is_same_v<VehicleRoute_i, TruckRoute> && std::is_same_v<VehicleRoute_j, DroneRoute>)        \
                        {                                                                                                              \
                            if (std::any_of(                                                                                           \
                                    customers_i.begin() + i, customers_i.begin() + (i + X),                                            \
                                    [&problem](const std::size_t &c) { return !problem->customers[c].dronable; }))                     \
                            {                                                                                                          \
                                continue;                                                                                              \
                            }                                                                                                          \
                        }                                                                                                              \
                                                                                                                                       \
                        bool ri_empty = (ri.size() == 2), rj_empty = (rj.size() == 2); /* Note: At least 1 flag is false */            \
                        if (ri_empty)                                                                                                  \
                        {                                                                                                              \
                            vehicle_routes_j[_vehicle_j][route_j] = VehicleRoute_j(rj);                                                \
                            vehicle_routes_i[_vehicle_i].erase(vehicle_routes_i[_vehicle_i].begin() + route_i);                        \
                        }                                                                                                              \
                        else if (rj_empty)                                                                                             \
                        {                                                                                                              \
                            vehicle_routes_i[_vehicle_i][route_i] = VehicleRoute_i(ri);                                                \
                            vehicle_routes_j[_vehicle_j].erase(vehicle_routes_j[_vehicle_j].begin() + route_j);                        \
                        }                                                                                                              \
                        else                                                                                                           \
                        {                                                                                                              \
                            vehicle_routes_i[_vehicle_i][route_i] = VehicleRoute_i(ri);                                                \
                            vehicle_routes_j[_vehicle_j][route_j] = VehicleRoute_j(rj);                                                \
                        }                                                                                                              \
                                                                                                                                       \
                        auto new_solution = this->construct(parent, truck_routes, drone_routes);                                       \
                        if ((aspiration_criteria(new_solution) || !this->is_tabu(customers_i[i], customers_j[j])) &&                   \
                            (result == nullptr || new_solution->cost() < result->cost()))                                              \
                        {                                                                                                              \
                            result = new_solution;                                                                                     \
                            tabu_pair = std::make_pair(customers_i[i], customers_j[j]);                                                \
                        }                                                                                                              \
                                                                                                                                       \
                        /* Restore */                                                                                                  \
                        vehicle_routes_i[_vehicle_i] = solution->vehicle_routes_i[_vehicle_i];                                         \
                        vehicle_routes_j[_vehicle_j] = solution->vehicle_routes_j[_vehicle_j];                                         \
                    }                                                                                                                  \
                }                                                                                                                      \
            }                                                                                                                          \
        }                                                                                                                              \
    }

                    if (vehicle_i < problem->trucks_count)
                    {
                        if (vehicle_j < problem->trucks_count)
                        {
                            MODIFY_ROUTES(truck_routes, truck_routes);
                        }
                        else
                        {
                            MODIFY_ROUTES(truck_routes, drone_routes);
                        }
                    }
                    else
                    {
                        if (vehicle_j < problem->trucks_count)
                        {
                            MODIFY_ROUTES(drone_routes, truck_routes);
                        }
                        else
                        {
                            MODIFY_ROUTES(drone_routes, drone_routes);
                        }
                    }

#undef MODIFY_ROUTES
                }
            }

            if constexpr (X == 0 || Y == 0)
            {
                constexpr std::size_t Z = X + Y;

#define APPEND_ROUTE(vehicle_routes)                                                                                      \
    {                                                                                                                     \
        for (std::size_t vehicle_i = 0; vehicle_i < vehicle_routes.size(); vehicle_i++)                                   \
        {                                                                                                                 \
            for (std::size_t route_i = 0; route_i < vehicle_routes[vehicle_i].size(); route_i++)                          \
            {                                                                                                             \
                using VehicleRoute_i = std::remove_cvref_t<decltype(vehicle_routes[vehicle_i][route_i])>;                 \
                VehicleRoute_i old_route(vehicle_routes[vehicle_i][route_i]);                                             \
                                                                                                                          \
                for (std::size_t vehicle_j = 0; vehicle_j < problem->trucks_count + problem->drones_count; vehicle_j++)   \
                {                                                                                                         \
                    const std::vector<std::size_t> &customers = solution->vehicle_routes[vehicle_i][route_i].customers(); \
                    for (std::size_t i = 1; i + Z < customers.size(); i++)                                                \
                    {                                                                                                     \
                        std::vector<std::size_t> new_customers(customers.begin(), customers.begin() + i);                 \
                        new_customers.insert(new_customers.end(), customers.begin() + (i + Z), customers.end());          \
                                                                                                                          \
                        std::vector<std::size_t> detached = {0};                                                          \
                        detached.insert(detached.end(), customers.begin() + i, customers.begin() + (i + Z));              \
                        detached.push_back(0);                                                                            \
                                                                                                                          \
                        if constexpr (std::is_same_v<VehicleRoute_i, TruckRoute>)                                         \
                        {                                                                                                 \
                            if (vehicle_j >= problem->trucks_count &&                                                     \
                                std::any_of(                                                                              \
                                    detached.begin() + 1, detached.end() - 1, /* excluding depot */                       \
                                    [&problem](const std::size_t &c) { return !problem->customers[c].dronable; }))        \
                            {                                                                                             \
                                continue;                                                                                 \
                            }                                                                                             \
                        }                                                                                                 \
                                                                                                                          \
                        /* Temporary modify */                                                                            \
                        if (new_customers.size() == 2)                                                                    \
                        {                                                                                                 \
                            if constexpr (std::is_same_v<VehicleRoute_i, TruckRoute>)                                     \
                            {                                                                                             \
                                if (vehicle_j < problem->trucks_count && vehicle_i == vehicle_j)                          \
                                {                                                                                         \
                                    continue;                                                                             \
                                }                                                                                         \
                            }                                                                                             \
                            else                                                                                          \
                            {                                                                                             \
                                if (vehicle_j >= problem->trucks_count && vehicle_i == vehicle_j - problem->trucks_count) \
                                {                                                                                         \
                                    continue;                                                                             \
                                }                                                                                         \
                            }                                                                                             \
                                                                                                                          \
                            vehicle_routes[vehicle_i].erase(vehicle_routes[vehicle_i].begin() + route_i);                 \
                        }                                                                                                 \
                        else                                                                                              \
                        {                                                                                                 \
                            vehicle_routes[vehicle_i][route_i] = VehicleRoute_i(new_customers);                           \
                        }                                                                                                 \
                                                                                                                          \
                        if (vehicle_j < problem->trucks_count)                                                            \
                        {                                                                                                 \
                            truck_routes[vehicle_j].push_back(TruckRoute(detached));                                      \
                        }                                                                                                 \
                        else                                                                                              \
                        {                                                                                                 \
                            drone_routes[vehicle_j - problem->trucks_count].push_back(DroneRoute(detached));              \
                        }                                                                                                 \
                                                                                                                          \
                        auto new_solution = this->construct(parent, truck_routes, drone_routes);                          \
                        if ((aspiration_criteria(new_solution) || !this->is_tabu(customers[i], 0)) &&                     \
                            (result == nullptr || new_solution->cost() < result->cost()))                                 \
                        {                                                                                                 \
                            result = new_solution;                                                                        \
                            tabu_pair = std::make_pair(customers[i], 0);                                                  \
                        }                                                                                                 \
                                                                                                                          \
                        /* Restore */                                                                                     \
                        if (new_customers.size() == 2)                                                                    \
                        {                                                                                                 \
                            vehicle_routes[vehicle_i].insert(vehicle_routes[vehicle_i].begin() + route_i, old_route);     \
                        }                                                                                                 \
                        else                                                                                              \
                        {                                                                                                 \
                            vehicle_routes[vehicle_i][route_i] = old_route;                                               \
                        }                                                                                                 \
                        if (vehicle_j < problem->trucks_count)                                                            \
                        {                                                                                                 \
                            truck_routes[vehicle_j].pop_back();                                                           \
                        }                                                                                                 \
                        else                                                                                              \
                        {                                                                                                 \
                            drone_routes[vehicle_j - problem->trucks_count].pop_back();                                   \
                        }                                                                                                 \
                    }                                                                                                     \
                }                                                                                                         \
            }                                                                                                             \
        }                                                                                                                 \
    }

                APPEND_ROUTE(truck_routes);
                APPEND_ROUTE(drone_routes);

#undef APPEND_ROUTE
            }

            return std::make_pair(result, tabu_pair);
        }
    };

    template <typename ST, std::size_t X, std::size_t Y>
    class MoveXY : public _BaseMoveXY<ST, X, Y>
    {
    protected:
        std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> intra_route(
            std::shared_ptr<ST> solution,
            const std::function<bool(std::shared_ptr<ST>)> &aspiration_criteria) override
        {
            auto problem = Problem::get_instance();
            auto parent = this->parent_ptr(solution);
            std::shared_ptr<ST> result;
            std::pair<std::size_t, std::size_t> tabu_pair;

            std::vector<std::vector<TruckRoute>> truck_routes(solution->truck_routes);
            std::vector<std::vector<DroneRoute>> drone_routes(solution->drone_routes);

#define MODIFY_ROUTES(vehicles_count, vehicle_routes, X, Y)                                                                           \
    {                                                                                                                                 \
        for (std::size_t index = 0; index < problem->vehicles_count; index++)                                                         \
        {                                                                                                                             \
            for (std::size_t route = 0; route < vehicle_routes[index].size(); route++)                                                \
            {                                                                                                                         \
                const std::vector<std::size_t> &customers = solution->vehicle_routes[index][route].customers();                       \
                for (std::size_t i = 1; i + 1 < customers.size(); i++)                                                                \
                {                                                                                                                     \
                    for (std::size_t j = i + X; j + Y < customers.size(); j++)                                                        \
                    {                                                                                                                 \
                        /* Temporary modify the route */                                                                              \
                        std::vector<std::size_t> new_customers(customers);                                                            \
                        if constexpr (X > Y)                                                                                          \
                        {                                                                                                             \
                            std::swap_ranges(new_customers.begin() + i, new_customers.begin() + i + Y, new_customers.begin() + j);    \
                            std::rotate(new_customers.begin() + i + Y, new_customers.begin() + i + X, new_customers.begin() + j + Y); \
                        }                                                                                                             \
                        else                                                                                                          \
                        {                                                                                                             \
                            std::swap_ranges(new_customers.begin() + i, new_customers.begin() + i + X, new_customers.begin() + j);    \
                            std::rotate(new_customers.begin() + i + X, new_customers.begin() + j + X, new_customers.begin() + j + Y); \
                        }                                                                                                             \
                                                                                                                                      \
                        using VehicleRoute = std::remove_cvref_t<decltype(vehicle_routes[index][route])>;                             \
                        vehicle_routes[index][route] = VehicleRoute(new_customers);                                                   \
                                                                                                                                      \
                        auto new_solution = this->construct(parent, truck_routes, drone_routes);                                      \
                        if ((aspiration_criteria(new_solution) || !this->is_tabu(customers[i], customers[j])) &&                      \
                            (result == nullptr || new_solution->cost() < result->cost()))                                             \
                        {                                                                                                             \
                            result = new_solution;                                                                                    \
                            tabu_pair = std::make_pair(customers[i], customers[j]);                                                   \
                        }                                                                                                             \
                                                                                                                                      \
                        /* Restore */                                                                                                 \
                        vehicle_routes[index][route] = solution->vehicle_routes[index][route];                                        \
                    }                                                                                                                 \
                }                                                                                                                     \
            }                                                                                                                         \
        }                                                                                                                             \
    }

            MODIFY_ROUTES(trucks_count, truck_routes, X, Y);
            MODIFY_ROUTES(drones_count, drone_routes, X, Y);
            if constexpr (X != Y)
            {
                MODIFY_ROUTES(trucks_count, truck_routes, Y, X);
                MODIFY_ROUTES(drones_count, drone_routes, Y, X);
            }

#undef MODIFY_ROUTES

            return std::make_pair(result, tabu_pair);
        }
    };

    template <typename ST, std::size_t X>
    class MoveXY<ST, X, 0> : public _BaseMoveXY<ST, X, 0>
    {
    protected:
        std::pair<std::shared_ptr<ST>, std::pair<std::size_t, std::size_t>> intra_route(
            std::shared_ptr<ST> solution,
            const std::function<bool(std::shared_ptr<ST>)> &aspiration_criteria) override
        {
            auto problem = Problem::get_instance();
            auto parent = this->parent_ptr(solution);
            std::shared_ptr<ST> result;
            std::pair<std::size_t, std::size_t> tabu_pair;

            std::vector<std::vector<TruckRoute>> truck_routes(solution->truck_routes);
            std::vector<std::vector<DroneRoute>> drone_routes(solution->drone_routes);

#define MODIFY_ROUTES(vehicles_count, vehicle_routes)                                                                             \
    {                                                                                                                             \
        for (std::size_t index = 0; index < problem->vehicles_count; index++)                                                     \
        {                                                                                                                         \
            for (std::size_t route = 0; route < vehicle_routes[index].size(); route++)                                            \
            {                                                                                                                     \
                const std::vector<std::size_t> &customers = solution->vehicle_routes[index][route].customers();                   \
                for (std::size_t i = 1; i + X < customers.size(); i++)                                                            \
                {                                                                                                                 \
                    for (std::size_t j = 1; j < i; j++)                                                                           \
                    {                                                                                                             \
                        /* Temporary modify the route */                                                                          \
                        std::vector<std::size_t> new_customers(customers);                                                        \
                        std::rotate(new_customers.begin() + j, new_customers.begin() + i, new_customers.begin() + (i + X));       \
                                                                                                                                  \
                        using VehicleRoute = std::remove_cvref_t<decltype(vehicle_routes[index][route])>;                         \
                        vehicle_routes[index][route] = VehicleRoute(new_customers);                                               \
                                                                                                                                  \
                        auto new_solution = this->construct(parent, truck_routes, drone_routes);                                  \
                        if ((aspiration_criteria(new_solution) || !this->is_tabu(customers[i], customers[j])) &&                  \
                            (result == nullptr || new_solution->cost() < result->cost()))                                         \
                        {                                                                                                         \
                            result = new_solution;                                                                                \
                            tabu_pair = std::make_pair(customers[i], customers[j]);                                               \
                        }                                                                                                         \
                                                                                                                                  \
                        /* Restore */                                                                                             \
                        vehicle_routes[index][route] = solution->vehicle_routes[index][route];                                    \
                    }                                                                                                             \
                                                                                                                                  \
                    for (std::size_t j = i + X; j + 1 < customers.size(); j++)                                                    \
                    {                                                                                                             \
                        /* Temporary modify the route */                                                                          \
                        std::vector<std::size_t> new_customers(customers);                                                        \
                        std::rotate(new_customers.begin() + i, new_customers.begin() + (i + X), new_customers.begin() + (j + 1)); \
                                                                                                                                  \
                        using VehicleRoute = std::remove_cvref_t<decltype(vehicle_routes[index][route])>;                         \
                        vehicle_routes[index][route] = VehicleRoute(new_customers);                                               \
                                                                                                                                  \
                        auto new_solution = this->construct(parent, truck_routes, drone_routes);                                  \
                        if ((aspiration_criteria(new_solution) || !this->is_tabu(customers[i], customers[j])) &&                  \
                            (result == nullptr || new_solution->cost() < result->cost()))                                         \
                        {                                                                                                         \
                            result = new_solution;                                                                                \
                            tabu_pair = std::make_pair(customers[i], customers[j]);                                               \
                        }                                                                                                         \
                                                                                                                                  \
                        /* Restore */                                                                                             \
                        vehicle_routes[index][route] = solution->vehicle_routes[index][route];                                    \
                    }                                                                                                             \
                }                                                                                                                 \
            }                                                                                                                     \
        }                                                                                                                         \
    }

            MODIFY_ROUTES(trucks_count, truck_routes);
            MODIFY_ROUTES(drones_count, drone_routes);

#undef MODIFY_ROUTES

            return std::make_pair(result, tabu_pair);
        }
    };
}
