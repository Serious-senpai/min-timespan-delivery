#pragma once

#include "format.hpp"

namespace utils
{
    template <typename T, std::enable_if_t<std::is_arithmetic_v<T>, bool> = true>
    T pow2(const T &value)
    {
        return value * value;
    }

    template <typename T>
    T pow3(const T &value)
    {
        return value * pow2(value);
    }

    template <typename _Iterator>
    using _iterator_category_t = typename std::iterator_traits<_Iterator>::iterator_category;

    template <typename _InputIterator>
    using is_input_iterator_t = std::enable_if_t<std::is_convertible_v<_iterator_category_t<_InputIterator>, std::input_iterator_tag>, void>;

    template <typename T, std::enable_if_t<std::is_arithmetic_v<T>, bool> = true>
    T sqrt(const T &value)
    {
        if (value < 0)
        {
            throw std::out_of_range(format("Attempted to calculate square root of %s < 0", std::to_string(value).c_str()));
        }

        if (value == 0)
        {
            return 0;
        }

        T low = 0, high = std::max(static_cast<T>(1), value), accuracy = 1;
        if constexpr (std::is_floating_point_v<T>)
        {
            accuracy = 1.0e-6;
        }

        while (high - low > accuracy)
        {
            double mid = (low + high) / 2;
            if (mid * mid < value)
            {
                low = mid;
            }
            else
            {
                high = mid;
            }
        }

        return high;
    }

    template <typename T>
    T distance(const T &dx, const T &dy)
    {
        return sqrt(pow2(dx) + pow2(dy));
    }
}
