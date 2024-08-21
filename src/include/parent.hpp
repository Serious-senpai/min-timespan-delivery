#pragma once

#include "solutions.hpp"

namespace d2d
{
    template <typename ST>
    class ParentInfo
    {
    public:
        const std::shared_ptr<ST> ptr;
        const std::string label;

        ParentInfo(const std::shared_ptr<ST> ptr, const std::string &label) : ptr(ptr), label(label) {}
    };
}
