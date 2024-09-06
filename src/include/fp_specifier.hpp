#pragma once

#include "wrapper.hpp"

namespace utils
{
    template <double _Limit = 1e6>
    const char *fp_format_specifier(const double &value)
    {
        return value > _Limit ? "%.2e" : "%.2lf";
    }

    template <typename T, double _Limit = 1e6>
    const char *fp_format_specifier(const FloatingPointWrapper<T> &wrapper)
    {
        return wrapper.value > _Limit ? "%.2e" : "%.2lf";
    }
}
