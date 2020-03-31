//
// Created by james on 3/29/20.
//

#ifndef SRC_PIDFILTERS_H
#define SRC_PIDFILTERS_H

#include "Filter.h"
#include <math.h>

class DerivativeFilter : public Filter
{
public:
    //////////////////////////////////////////////////////////
    /// @brief The derivative filter takes three arguments
    ///        and then calculates the correct coefficients
    ///
    /// @param Ts: The sample period in seconds
    /// @param fc: The cutoff frequency in Hz
    /// @param Kd: The damping gain (unitless) - default is 1.0
    ///
    ////////////////////////////////////////////////////////////
    DerivativeFilter(double Ts, double fc, double Kd);

    //////////////////////////////////////////////////////////
    /// @brief The derivative filter takes three arguments
    ///        and then calculates the correct coefficients
    ///        This default constructor has hardcoded params
    ///
    ////////////////////////////////////////////////////////////
    DerivativeFilter() : DerivativeFilter(0.002, 50, 1.0){};

    void resetFilterParameters(double Ts, double fc, double Kd);
};

class IntegralFilter : public Filter
{
public:
    //////////////////////////////////////////////////////////
    /// @brief The integral filter takes two arguments
    ///        and then calculates the correct coefficients
    ///
    /// @param Ts: The sample period in seconds
    /// @param Ki: The integral gain (unitless) - default is 1.0
    ///
    ////////////////////////////////////////////////////////////
    IntegralFilter(double Ts, double Ki);

    //////////////////////////////////////////////////////////
    /// @brief The high pass filter takes two arguments
    ///        and then calculates the correct coefficients
    ///        This default constructor has hardcoded params
    ///
    ////////////////////////////////////////////////////////////
    IntegralFilter() : IntegralFilter(0.002, 1.0){};

    void resetFilterParameters(double Ts, double Ki);
};

#endif //SRC_PIDFILTERS_H
