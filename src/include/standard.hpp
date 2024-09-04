#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <vector>

#if defined(_WIN32) && !defined(WIN32)
#define WIN32
#endif

#if defined(WIN32)
#define WIN32_LEAN_AND_MEAN // otherwise "rad1" macro in dlgs.h clashes with "rad1" variable in ALGLIB optimization.h
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#include <sys/ioctl.h>
#endif

#include <cxxabi.h>

#include <dataanalysis.h>

namespace std
{
    /**
     * @brief This does exactly what you think it does
     *
     * @param _x The first value
     * @param _y The second value
     * @param _z The third value
     * @return The minimum of the 3 values
     */
    template <typename T>
    const T &min(const T &_x, const T &_y, const T &_z)
    {
        return min(_x, min(_y, _z));
    }

    /**
     * @brief This does exactly what you think it does
     *
     * @param _x The first value
     * @param _y The second value
     * @param _z The third value
     * @param _a The fourth value
     * @return The minimum of the 4 values
     */
    template <typename T>
    const T &min(const T &_x, const T &_y, const T &_z, const T &_a)
    {
        return min(_x, min(_y, _z, _a));
    }

    template <typename _ForwardIterator>
    void __list_elements(ostream &stream, const _ForwardIterator &_begin, const _ForwardIterator &_end)
    {
        for (auto iter = _begin; iter != _end; iter++)
        {
            stream << *iter;
            if (next(iter) != _end)
            {
                stream << ", ";
            }
        }
    }

    template <typename T, std::size_t N>
    ostream &operator<<(ostream &stream, const array<T, N> &_v)
    {
        stream << "[";
        __list_elements(stream, _v.begin(), _v.end());
        stream << "]";

        return stream;
    }

    template <typename T>
    ostream &operator<<(ostream &stream, const vector<T> &_v)
    {
        stream << "[";
        __list_elements(stream, _v.begin(), _v.end());
        stream << "]";

        return stream;
    }

    template <typename T>
    ostream &operator<<(ostream &stream, const list<T> &_l)
    {
        stream << "[";
        __list_elements(stream, _l.begin(), _l.end());
        stream << "]";

        return stream;
    }

    template <typename T, typename Compare, typename Alloc>
    ostream &operator<<(ostream &stream, const set<T, Compare, Alloc> &_s)
    {
        stream << "{";
        __list_elements(stream, _s.begin(), _s.end());
        stream << "}";

        return stream;
    }

    template <typename T, typename Compare, typename Alloc>
    ostream &operator<<(ostream &stream, const multiset<T, Compare, Alloc> &_s)
    {
        stream << "{";
        __list_elements(stream, _s.begin(), _s.end());
        stream << "}";

        return stream;
    }

    template <typename T1, typename T2, typename Compare, typename Alloc>
    ostream &operator<<(ostream &stream, const map<T1, T2, Compare, Alloc> &_m)
    {
        vector<pair<T1, T2>> _v(_m.begin(), _m.end());

        stream << "{";
        for (auto iter = _m.begin(); iter != _m.end(); iter++)
        {
            stream << iter->first << ": " << iter->second;
            if (next(iter) != _m.end())
            {
                stream << ", ";
            }
        }
        stream << "}";

        return stream;
    }

    template <typename T1, typename T2>
    ostream &operator<<(ostream &stream, const pair<T1, T2> &_p)
    {
        return stream << "(" << _p.first << ", " << _p.second << ")";
    }
}
