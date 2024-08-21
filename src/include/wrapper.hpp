#pragma once

#include "utils.hpp"

namespace utils
{
    template <typename T, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
    class FloatingPointWrapper
    {
    public:
        T value;

        FloatingPointWrapper(const T &value) : value(value) {}

        bool operator==(const FloatingPointWrapper<T> &other) const
        {
            return approximate(value, other.value);
        }

        bool operator!=(const FloatingPointWrapper<T> &other) const
        {
            return !(*this == other);
        }

        bool operator<(const FloatingPointWrapper<T> &other) const
        {
            return value + TOLERANCE < other.value;
        }

        bool operator>(const FloatingPointWrapper<T> &other) const
        {
            return value - TOLERANCE > other.value;
        }

        bool operator<=(const FloatingPointWrapper<T> &other) const
        {
            return *this < other || *this == other;
        }

        bool operator>=(const FloatingPointWrapper<T> &other) const
        {
            return *this > other || *this == other;
        }
    };
}

namespace std
{
    template <typename T>
    ostream &operator<<(ostream &stream, const utils::FloatingPointWrapper<T> &_w)
    {
        return stream << _w.value;
    }
}
