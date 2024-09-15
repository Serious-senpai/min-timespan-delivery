#pragma once

#include "wrapper.hpp"

namespace utils
{
    constexpr double __fp_format_specifier_default_limit = 1e8;

    template <typename T, T _Limit = static_cast<T>(__fp_format_specifier_default_limit), std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
    const char *fp_format_specifier(const T &value)
    {
        return value > _Limit ? "%.2e" : "%.2f";
    }

    template <typename T, T _Limit = static_cast<T>(__fp_format_specifier_default_limit)>
    const char *fp_format_specifier(const FloatingPointWrapper<T> &wrapper)
    {
        return fp_format_specifier<T, _Limit>(wrapper.value);
    }
}
