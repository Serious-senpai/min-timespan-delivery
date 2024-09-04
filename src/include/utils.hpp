#pragma once

#include "format.hpp"

#define TOLERANCE 1e-6

namespace utils
{
    class PerformanceBenchmark
    {
    private:
        const std::chrono::time_point<std::chrono::high_resolution_clock> start;
        const std::string message;

    public:
        PerformanceBenchmark(const std::string &message) : start(std::chrono::high_resolution_clock::now()), message(message) {}
        ~PerformanceBenchmark()
        {
            report();
        }

        void report() const
        {
            report(message);
        }

        void report(const std::string &message) const
        {
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
            std::cerr << message << " " << duration.count() << "ms\n";
        }
    };

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
    using is_input_iterator_t = std::enable_if_t<std::is_convertible_v<_iterator_category_t<_InputIterator>, std::input_iterator_tag>, bool>;

    template <typename _RT, typename _T>
    std::enable_if_t<
        std::is_same_v<std::remove_cvref_t<_RT>, std::remove_cvref_t<_T>>,
        std::conditional_t<std::is_const_v<_T>, const _RT, _RT>> &
    match_type(_T &t)
    {
        return t;
    }

    template <typename _RT, typename _T, typename... Args>
    auto &match_type(_T &t1, Args &...args)
    {
        if constexpr (std::is_same_v<std::remove_cvref_t<_RT>, std::remove_cvref_t<_T>>)
        {
            return t1;
        }
        else
        {
            return match_type<_RT>(args...);
        }
    }

    template <bool _Cond, typename _IfTrue, typename _IfFalse>
    const std::conditional_t<_Cond, _IfTrue, _IfFalse> &ternary(const _IfTrue &if_true, const _IfFalse &if_false)
    {
        if constexpr (_Cond)
        {
            return if_true;
        }
        else
        {
            return if_false;
        }
    }

    template <typename T>
    std::string type()
    {
        const char *mangled = typeid(T).name();
        int status;
        std::unique_ptr<char> ptr(abi::__cxa_demangle(mangled, nullptr, nullptr, &status));

        std::string name(status == 0 ? ptr.get() : mangled);
        if constexpr (std::is_const_v<T>)
        {
            name = "const " + name;
        }

        if constexpr (std::is_volatile_v<T>)
        {
            name = "volatile " + name;
        }

        if constexpr (std::is_pointer_v<T>)
        {
            name += "*";
        }

        if constexpr (std::is_lvalue_reference_v<T>)
        {
            name += "&";
        }
        else if constexpr (std::is_rvalue_reference_v<T>)
        {
            name += "&&";
        }

        return name;
    }

    template <typename T, std::enable_if_t<std::is_arithmetic_v<T>, bool> = true>
    T sqrt(const T &value)
    {
        if (value < static_cast<T>(0))
        {
            throw std::out_of_range(format("Attempted to calculate square root of %s < 0", std::to_string(value).c_str()));
        }

        T low = 0, high = std::max(static_cast<T>(1), value), error = 1;
        if constexpr (std::is_floating_point_v<T>)
        {
            error = 1.0e-7;
        }

        if (high * high == value)
        {
            return high;
        }

        while (high - low > error)
        {
            double mid = (low + high) / 2;
            if (mid * mid > value)
            {
                high = mid;
            }
            else
            {
                low = mid;
            }
        }

        return low;
    }

    template <typename T>
    T distance(const T &dx, const T &dy)
    {
        return sqrt(pow2(dx) + pow2(dy));
    }

    template <typename T, std::enable_if_t<std::is_arithmetic_v<T>, bool> = true>
    T abs(const T &value)
    {
        return value > 0 ? value : -value;
    }

    template <typename T, std::enable_if_t<std::negation_v<std::is_floating_point<T>>, bool> = true>
    bool approximate(const T &first, const T &second)
    {
        return first == second;
    }

    template <typename T, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
    bool approximate(const T &first, const T &second)
    {
        return abs(first - second) < TOLERANCE;
    }

    template <typename T>
    bool approximate(const std::vector<T> &first, const std::vector<T> &second)
    {
        return std::equal(
            first.begin(), first.end(),
            second.begin(), second.end(),
            [](const T &first, const T &second)
            {
                return approximate(first, second);
            });
    }

    /**
     * @brief Get the size of the console window using
     * [`GetConsoleScreenBufferInfo`](https://learn.microsoft.com/en-us/windows/console/getconsolescreenbufferinfo)
     * or [`ioctl`](https://man7.org/linux/man-pages/man2/ioctl.2.html).
     *
     * @param _stdout Whether to get the size of the standard output or standard error.
     * @note In case it is not possible to get the console size, `std::runtime_error` is thrown.
     * @return The number of columns and rows, respectively.
     */
    std::pair<unsigned short, unsigned short> get_console_size(const bool &_stdout = true)
    {
#if defined(WIN32)
        CONSOLE_SCREEN_BUFFER_INFO info;
        if (!GetConsoleScreenBufferInfo(GetStdHandle(_stdout ? STD_OUTPUT_HANDLE : STD_ERROR_HANDLE), &info))
        {
            throw std::runtime_error("GetConsoleScreenBufferInfo ERROR");
        }

        SHORT columns = info.srWindow.Right - info.srWindow.Left + 1,
              rows = info.srWindow.Bottom - info.srWindow.Top + 1;

        return std::make_pair(columns, rows);

#elif defined(__linux__)
        struct winsize w;
        if (ioctl(_stdout ? STDOUT_FILENO : STDERR_FILENO, TIOCGWINSZ, &w) == -1)
        {
            throw std::runtime_error("ioctl ERROR");
        }

        return std::make_pair(w.ws_col, w.ws_row);
#else
        throw std::runtime_error("Unsupported platform");
#endif
    }
}
