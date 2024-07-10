#pragma once

#include "utils.hpp"

namespace utils
{
    /**
     * @brief Implementation of a [Fenwick tree](https://en.wikipedia.org/wiki/Fenwick_tree)
     *
     * @tparam T An arithmetic type
     */
    template <typename T, std::enable_if_t<std::is_arithmetic_v<T>, bool> = true>
    class FenwickTree
    {
    private:
        // The underlying array, its size is always `_tree.size() - 1`
        std::vector<T> _array;

        // The Fenwick tree representation, its size is always `_array.size() + 1`
        std::vector<T> _tree;

        T _sum(std::size_t _tree_begin, std::size_t _tree_end) const
        {
            // Return result as `add - sub` instead of updating a single `result` variable
            // since `result` may be negative when T is unsigned.
            T add = 0, sub = 0;
            while (_tree_begin != _tree_end)
            {
                if (_tree_end > _tree_begin)
                {
                    add += _tree[_tree_end];
                    _tree_end ^= _tree_end & -_tree_end;
                }
                else
                {
                    sub += _tree[_tree_begin];
                    _tree_begin ^= _tree_begin & -_tree_begin;
                }
            }

            return add - sub;
        }

    public:
        /** @brief Construct a new FenwickTree object */
        FenwickTree()
        {
            _tree.push_back(static_cast<T>(0));
        }

        /**
         * @brief Construct a new FenwickTree object from the range [begin, end)
         *
         * @param begin An iterator to the beginning of the range
         * @param end An iterator past the end of the range
         */
        template <typename _InputIterator, typename = is_input_iterator_t<_InputIterator>>
        FenwickTree(const _InputIterator &begin, const _InputIterator &end) : FenwickTree()
        {
            _array.insert(_array.end(), begin, end);
            _tree.reserve(_array.size() + 1);
            for (const T &value : _array)
            {
                push_back(value);
            }
        }

        /**
         * @brief Get the value at the specified index of the underlying array.
         *
         * @param index The index to get the value from (0-based)
         */
        T get(const std::size_t &index) const
        {
            return _array[index];
        }

        /** @brief Get the size of the underlying array */
        std::size_t size() const
        {
            return _array.size();
        }

        /**
         * @brief Calculate the sum over the range `[offset, offset + length)` of
         * the underlying array.
         *
         * If the specified range is invalid, the result is undefined.
         *
         * @param offset The starting index of the subarray (0-based)
         * @param length The length of the subarray
         * @return The sum of the subarray
         */
        T sum(const std::size_t &offset, const std::size_t &length) const
        {
            return _sum(offset, offset + length);
        }

        /** @brief Calculate the sum of the array */
        T sum() const
        {
            return _sum(0, size());
        }

        /**
         * @brief Update a specific element of the underlying array
         *
         * @param index The index to update (0-based)
         * @param value The new value
         */
        void set(const std::size_t &index, const T &value)
        {
            const T diff = value - _array[index];
            _array[index] = value;
            for (std::size_t i = index + 1; i < _tree.size(); i = (i | (i - 1)) + 1)
            {
                if constexpr (std::is_unsigned_v<T>)
                {
                    _tree[i] += value;
                    _tree[i] -= _array[index];
                }
                else
                {
                    _tree[i] += diff;
                }
            }
        }

        /**
         * @brief Append a value to the end of the underlying array
         *
         * @param value The value to append
         */
        void push_back(const T &value)
        {
            _array.push_back(value);

            std::size_t index = _tree.size();
            _tree.push_back(value + _sum(index ^ (index & -index), index - 1));
        }

        /** @brief Return a const_iterator to the beginning of underlying array */
        std::vector<T>::const_iterator begin() const
        {
            return _array.cbegin();
        }

        /** @brief Return a const_iterator past the end of underlying array */
        std::vector<T>::const_iterator end() const
        {
            return _array.cend();
        }

        /** @brief Return a const_iterator to the beginning of underlying array */
        std::vector<T>::const_iterator cbegin() const
        {
            return _array.cbegin();
        }

        /** @brief Return a const_iterator past the end of underlying array */
        std::vector<T>::const_iterator cend() const
        {
            return _array.cend();
        }
    };
}
