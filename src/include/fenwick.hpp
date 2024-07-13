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
        /**
         * @brief Construct a new FenwickTree object.
         * @note Time complexity `O(1)`
         */
        FenwickTree()
        {
            _tree.push_back(static_cast<T>(0));
        }

        /**
         * @brief Construct a new FenwickTree object from the range [begin, end).
         *
         * @param begin An iterator to the beginning of the range
         * @param end An iterator past the end of the range
         * @note Time complexity `O(nlogn)`, where `n` is the number of elements
         * between `begin` and `end`.
         */
        template <typename _InputIterator, is_input_iterator_t<_InputIterator> = true>
        FenwickTree(const _InputIterator &begin, const _InputIterator &end) : FenwickTree()
        {
            for (auto iter = begin; iter != end; iter++)
            {
                push_back(*iter);
            }
        }

        /**
         * @brief Construct a new FenwickTree object from the "prefix" of another
         * FenwickTree object.
         *
         * Although this constructor results in an object identical to
         * `FenwickTree<T>(other.begin(), other.begin() + n)`, it offers
         * a more favorable time complexity.
         *
         * @param other The other FenwickTree object
         * @param n The length of the prefix to copy
         * @note Time complexity `O(n)`
         */
        FenwickTree(const FenwickTree<T> &other, const std::size_t &n)
        {
            if (other.size() < n)
            {
                throw std::out_of_range(format("Cannot copy %lu elements from a FenwickTree of size %lu", n, other.size()));
            }

            _array.insert(_array.end(), other._array.begin(), other._array.begin() + n);
            _tree.insert(_tree.end(), other._tree.begin(), other._tree.begin() + n + 1);
        }

        /**
         * @brief Get the value at the specified index of the underlying array.
         *
         * @param index The index to get the value from (0-based)
         * @note Time complexity `O(1)`
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
         * @note Time complexity `O(logn)`, where `n` is the size of the underlying array.
         */
        T sum(const std::size_t &offset, const std::size_t &length) const
        {
            return _sum(offset, offset + length);
        }

        /**
         * @brief Calculate the sum of the underlying array.
         *
         * This is a shorthand for `sum(0, size())`.
         */
        T sum() const
        {
            return _sum(0, size());
        }

        /**
         * @brief Update a specific element of the underlying array
         *
         * @param index The index to update (0-based)
         * @param value The new value
         * @note Time complexity `O(logn)`, where `n` is the size of the underlying array.
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

        /** @brief Attempt to preallocate enough memory for specified number of elements. */
        void reserve(const std::size_t &size)
        {
            _array.reserve(size);
            _tree.reserve(size + 1);
        }

        /**
         * @brief Append a value to the end of the underlying array
         *
         * @param value The value to append
         * @note Time complexity `O(logn)`, where `n` is the size of the underlying array.
         */
        void push_back(const T &value)
        {
            _array.push_back(value);

            std::size_t index = _tree.size();
            _tree.push_back(value + _sum(index ^ (index & -index), index - 1));
        }

        /**
         * @brief Remove the last element of the underlying array. No data is returned.
         */
        void pop_back()
        {
            if (_array.empty())
            {
                throw std::out_of_range("Cannot pop from an empty FenwickTree");
            }

            _array.pop_back();
            _tree.pop_back();
        }

        /** @brief Alias of `std::vector<T>::const_iterator` */
        using const_iterator = typename std::vector<T>::const_iterator;

        /** @brief Return a const_iterator to the beginning of underlying array */
        const_iterator begin() const
        {
            return _array.cbegin();
        }

        /** @brief Return a const_iterator past the end of underlying array */
        const_iterator end() const
        {
            return _array.cend();
        }

        /** @brief Return a const_iterator to the beginning of underlying array */
        const_iterator cbegin() const
        {
            return _array.cbegin();
        }

        /** @brief Return a const_iterator past the end of underlying array */
        const_iterator cend() const
        {
            return _array.cend();
        }

        /** @brief Get a const reference of the underlying array */
        const std::vector<T> &array() const
        {
            return _array;
        }

        /** @brief Get the first element of the underlying array */
        T front() const
        {
            return _array.front();
        }

        /** @brief Get the last element of the underlying array */
        T back() const
        {
            return _array.back();
        }
    };

    template <typename T>
    bool operator==(const FenwickTree<T> &first, const FenwickTree<T> &second)
    {
        return first.array() == second.array();
    }

    template <typename T>
    bool operator!=(const FenwickTree<T> &first, const FenwickTree<T> &second)
    {
        return first.array() != second.array();
    }

    template <typename T>
    bool operator<(const FenwickTree<T> &first, const FenwickTree<T> &second)
    {
        return first.array() < second.array();
    }

    template <>
    bool approximate(const FenwickTree<double> &first, const FenwickTree<double> &second)
    {
        return approximate(first.array(), second.array());
    }
}

namespace std
{
    template <typename T>
    ostream &operator<<(ostream &stream, const utils::FenwickTree<T> &tree)
    {
        return stream << tree.array();
    }
}
