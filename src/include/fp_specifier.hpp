#pragma once

#include "wrapper.hpp"

namespace utils
{
    template <typename T, T _Limit = static_cast<T>(1e6), std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
    const char *fp_format_specifier(const T &value)
    {
        return value > _Limit ? "%.2e" : "%.2f";
    }

    template <typename T, T _Limit = static_cast<T>(1e6)>
    const char *fp_format_specifier(const FloatingPointWrapper<T> &wrapper)
    {
        return wrapper.value > _Limit ? "%.2e" : "%.2f";
    }
}
