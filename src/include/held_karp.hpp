#pragma once

#include "standard.hpp"
#include "utils.hpp"

namespace utils
{
    std::pair<double, std::size_t> __held_karp_solve(
        const std::size_t &n,
        const std::size_t &bitmask,
        const std::size_t &city,
        const std::function<double(const std::size_t &, const std::size_t &)> &distance,
        std::vector<std::vector<std::pair<double, std::size_t>>> &dp)
    {
        if (dp[bitmask][city].first != -1.0)
        {
            return dp[bitmask][city];
        }

        if (bitmask & (1u << city))
        {
            return dp[bitmask][city] = __held_karp_solve(n, bitmask & ~(1u << city), city, distance, dp);
        }

        if (bitmask & 1u)
        {
            return dp[bitmask][city] = __held_karp_solve(n, bitmask ^ 1u, city, distance, dp);
        }

        std::pair<double, std::size_t> result = {-1.0, n};
        for (std::size_t i = 1; i < n; i++)
        {
            if (bitmask & (1u << i))
            {
                auto before = __held_karp_solve(n, bitmask & ~(1u << i), i, distance, dp);
                double d = before.first + distance(i, city);
                if (d < result.first || result.first == -1.0)
                {
                    result = {d, i};
                }
            }
        }

        return dp[bitmask][city] = result;
    }

    std::pair<double, std::vector<std::size_t>> __held_karp(
        const std::size_t &n,
        const std::function<double(const std::size_t &, const std::size_t &)> &distance)
    {
        // https://en.wikipedia.org/wiki/Held-Karp_algorithm
        std::vector<std::vector<std::pair<double, std::size_t>>> dp(1u << n, std::vector<std::pair<double, std::size_t>>(n, {-1.0, n}));
        for (std::size_t end = 1; end < n; end++)
        {
            dp[0][end] = {distance(0, end), 0};
        }

        std::size_t path_end = 0, bitmask = (1u << n) - 2;
        std::pair<double, std::size_t> distance_end = {1.0e+9, -1};
        for (std::size_t end = 1; end < n; end++)
        {
            auto r = __held_karp_solve(n, bitmask, end, distance, dp);
            r.first += distance(0, end);
            if (r < distance_end)
            {
                distance_end = r;
                path_end = end;
            }
        }

        bitmask &= ~(1u << path_end);
        std::vector<std::size_t> path = {0, path_end};
        while (bitmask > 0)
        {
            auto r = __held_karp_solve(n, bitmask, path_end, distance, dp);
            path_end = r.second;
            bitmask &= ~(1u << path_end);
            path.push_back(path_end);
        }

        return {distance_end.first, path};
    }

    std::pair<double, std::vector<std::size_t>> held_karp_algorithm(
        const std::size_t &n,
        const std::function<double(const std::size_t &, const std::size_t &)> &distance)
    {
        if (n == 0)
        {
            throw std::invalid_argument("Empty TSP problem");
        }

        if (n == 1)
        {
            std::vector<std::size_t> path = {0};
            return {0, path};
        }

        return __held_karp(n, distance);
    }

    std::pair<double, std::vector<std::size_t>> nearest_heuristic(
        const std::size_t &n,
        const std::function<double(const std::size_t &, const std::size_t &)> &distance)
    {
        std::vector<std::size_t> path(n);
        std::iota(path.begin(), path.end(), 0);

        for (auto iter = path.begin(); iter != path.end(); iter++)
        {
            auto nearest = std::min_element(
                iter + 1, path.end(),
                [&distance, &iter](const std::size_t &i, const std::size_t &j)
                {
                    return distance(*iter, i) < distance(*iter, j);
                });

            if (nearest != path.end())
            {
                std::iter_swap(iter + 1, nearest);
            }
        }

        double d = 0;
        for (std::size_t i = 0; i < n - 1; i++)
        {
            d += distance(path[i], path[i + 1]);
        }

        return std::make_pair(d, path);
    }
}
