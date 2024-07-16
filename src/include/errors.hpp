#pragma once

#include "format.hpp"

namespace d2d
{
    class BaseException : public std::exception
    {
    public:
        const std::string message;

        BaseException(const std::string &message) : message(message) {}

        const char *what() const noexcept
        {
            return message.c_str();
        }
    };

    class NonDronable : public BaseException
    {
    public:
        const std::size_t customer;

        NonDronable(const std::size_t &customer)
            : BaseException(utils::format("Customer %lu is not allowed to be served by drone", customer)),
              customer(customer) {}
    };
}
