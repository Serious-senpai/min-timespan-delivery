#pragma once

#include "utils.hpp"

namespace utils
{
    class BitVector
    {
    private:
        static const uint64_t _BIT = 1;

        std::vector<uint64_t> _arr;
        std::size_t _size;

        BitVector(const std::vector<uint64_t> &arr, const std::size_t &size) : _arr(arr), _size(size) {}

    public:
        BitVector(const std::size_t &size) : _arr((size + 63) >> 6), _size(size) {}

        std::size_t size() const
        {
            return _size;
        }

        void set(const std::size_t &index)
        {
            _arr[index >> 6] |= (_BIT << (index & 63));
        }

        void clear(const std::size_t &index)
        {
            _arr[index >> 6] &= ~(_BIT << (index & 63));
        }

        void clear()
        {
            _arr.clear();
            _size = 0;
        }

        void push_back(const bool &value)
        {
            if ((_size & 63) == 0)
            {
                _arr.push_back(0);
            }

            if (value)
            {
                _arr.back() |= (_BIT << (_size & 63));
            }
            else
            {
                _arr.back() &= ~(_BIT << (_size & 63));
            }

            _size++;
        }

        void pop_back()
        {
            _size--;
            if ((_size & 63) == 0)
            {
                _arr.pop_back();
            }
            else
            {
                _arr[_size >> 6] &= ~(_BIT << (_size & 63));
            }
        }

        bool empty() const
        {
            return _size == 0;
        }

        int popcount() const
        {
            return std::accumulate(
                _arr.begin(), _arr.end(),
                0,
                [](const int &acc, const uint64_t &value)
                {
                    return acc + std::popcount(value);
                });
        }

        bool operator[](const std::size_t &index) const
        {
            return (_arr[index >> 6] & (_BIT << (index & 63))) != 0;
        }

        BitVector operator^(const BitVector &other) const
        {
            // Most likely a compiler bug? See https://godbolt.org/z/rbqsnhEsP for an example
            // const auto [l, r] = std::minmax(_arr.size(), other._arr.size());
            const std::size_t l = std::min(_arr.size(), other._arr.size()),
                              r = std::max(_arr.size(), other._arr.size());
            std::vector<uint64_t> result(r);

            for (std::size_t i = 0; i < l; i++)
            {
                result[i] = _arr[i] ^ other._arr[i];
            }

            if (_arr.size() == r)
            {
                for (std::size_t i = l; i < r; i++)
                {
                    result[i] = _arr[i];
                }
            }

            if (other._arr.size() == r)
            {
                for (std::size_t i = l; i < r; i++)
                {
                    result[i] = other._arr[i];
                }
            }

            return BitVector(result, std::max(_size, other._size));
        }
    };
}
