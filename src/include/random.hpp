#pragma once

#include "format.hpp"

namespace utils
{
    /** @brief A random number generator */
    std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());

    /**
     * @brief Generate a random number in the range `[l, r]`
     *
     * @param l The left endpoint
     * @param r The right endpoint
     * @return A random number
     */
    template <typename T, std::enable_if_t<std::is_arithmetic_v<T>, bool> = true>
    T random(const T &l, const T &r)
    {
        std::conditional_t<std::is_floating_point_v<T>, std::uniform_real_distribution<T>, std::uniform_int_distribution<T>> unif(l, r);
        return unif(rng);
    }

    /**
     * @brief Perform a weighted random selection
     *
     * @param weights The weights of the elements
     * @param count The number of elements to select
     * @return The index of selected elements
     */
    std::vector<std::size_t> weighted_random(const std::vector<double> &weights, const std::size_t count = 1)
    {
        std::size_t n = weights.size();
        if (count > n)
        {
            throw std::invalid_argument(format("Argument \"count\" exceeded the number of weights (%d > %d)", count, weights.size()));
        }

        double sum_weight = 0;
        for (auto weight : weights)
        {
            sum_weight += weight;
            if (weight < 0)
            {
                throw std::invalid_argument(format("Received weight %lf < 0.0", weight));
            }
        }

        std::set<std::size_t> results;

        std::size_t limit = std::min(count, n - count);
        while (results.size() < limit)
        {
            double value = random(0.0, sum_weight);
            for (std::size_t index = 0; index < n; index++)
            {
                if (!results.contains(index))
                {
                    value -= weights[index];
                    if (value <= 0)
                    {
                        results.insert(index);
                        sum_weight -= weights[index];
                        break;
                    }
                }
            }
        }

        if (limit == count)
        {
            return std::vector<std::size_t>(results.begin(), results.end());
        }

        std::vector<std::size_t> returns;
        for (std::size_t index = 0; index < n; index++)
        {
            if (!results.contains(index))
            {
                returns.push_back(index);
            }
        }

        return returns;
    }

    /**
     * @brief Select a random element from a vector
     *
     * @param v The vector to select from
     * @return A reference to the selected element
     */
    template <typename T>
    const T &random_element(const std::vector<T> &v)
    {
        if (v.empty())
        {
            throw std::runtime_error("Cannot select an element from an empty vector");
        }

        return v[random(static_cast<std::size_t>(0), v.size() - 1)];
    }
}
