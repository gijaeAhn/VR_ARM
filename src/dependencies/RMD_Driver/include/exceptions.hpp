//
// Created by gj on 24. 3. 4.
//

#ifndef RMD_DRIVER_EXCEPTIONS_HPP
#define RMD_DRIVER_EXCEPTIONS_HPP
#pragma once

#include <stdexcept>


namespace myactuator_rmd {

    /**\class Exception
     * \brief
     *    Exception class for driver-level errors
    */
    class Exception: public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

    /**\class ProtocolException
     * \brief
     *    Exception class for driver protocol parsing error
    */
    class ProtocolException: public Exception {
    public:
        using Exception::Exception;
    };

    /**\class ValueRangeException
     * \brief
     *    Exception class for values that are outside their expected range
    */
    class ValueRangeException: public Exception {
    public:
        using Exception::Exception;
    };




#endif //RMD_DRIVER_EXCEPTIONS_HPP
