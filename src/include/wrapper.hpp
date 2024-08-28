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

        template <typename _FT, std::enable_if_t<std::is_floating_point_v<_FT>, bool> = true>
        FloatingPointWrapper &operator=(const _FT &other)
        {
            value = other;
            return *this;
        }

        template <typename _FT, std::enable_if_t<std::is_floating_point_v<_FT>, bool> = true>
        FloatingPointWrapper &operator+(const _FT &other)
        {
            value += other;
            return *this;
        }

        template <typename _FT, std::enable_if_t<std::is_floating_point_v<_FT>, bool> = true>
        FloatingPointWrapper &operator+(const FloatingPointWrapper<_FT> &other)
        {
            value += other.value;
            return *this;
        }

        template <typename _FT, std::enable_if_t<std::is_floating_point_v<_FT>, bool> = true>
        FloatingPointWrapper &operator-(const _FT &other)
        {
            value -= other;
            return *this;
        }

        template <typename _FT, std::enable_if_t<std::is_floating_point_v<_FT>, bool> = true>
        FloatingPointWrapper &operator-(const FloatingPointWrapper<_FT> &other)
        {
            value -= other.value;
            return *this;
        }

        template <typename _FT, std::enable_if_t<std::is_floating_point_v<_FT>, bool> = true>
        bool operator==(const _FT &other) const
        {
            return approximate(value, other);
        }

        template <typename _FT, std::enable_if_t<std::is_floating_point_v<_FT>, bool> = true>
        bool operator==(const FloatingPointWrapper<_FT> &other) const
        {
            return *this == other.value;
        }

        template <typename _FT, std::enable_if_t<std::is_floating_point_v<_FT>, bool> = true>
        bool operator<(const _FT &other) const
        {
            return value + TOLERANCE < other;
        }

        template <typename _FT, std::enable_if_t<std::is_floating_point_v<_FT>, bool> = true>
        bool operator<(const FloatingPointWrapper<_FT> &other) const
        {
            return *this < other.value;
        }

        template <typename _FT, std::enable_if_t<std::is_floating_point_v<_FT>, bool> = true>
        bool operator>(const _FT &other) const
        {
            return value - TOLERANCE > other;
        }

        template <typename _FT, std::enable_if_t<std::is_floating_point_v<_FT>, bool> = true>
        bool operator>(const FloatingPointWrapper<_FT> &other) const
        {
            return *this > other.value;
        }

        template <typename _FT>
        bool operator!=(const _FT &other) const
        {
            return !(*this == other);
        }

        template <typename _FT>
        bool operator<=(const _FT &other) const
        {
            return *this < other || *this == other;
        }

        template <typename _FT>
        bool operator>=(const _FT &other) const
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
