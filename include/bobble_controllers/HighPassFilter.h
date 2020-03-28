//
// Created by james on 3/27/20.
//

#ifndef SRC_HIGHPASSFILTER_H
#define SRC_HIGHPASSFILTER_H

#include "Filter.h"
#include <math.h>

class HighPassFilter : public Filter
{
public:
    //////////////////////////////////////////////////////////
    /// @brief The high pass filter takes three arguments
    ///        and then calculates the correct coefficients
    ///
    /// @param Ts: The sample period in seconds
    /// @param fc: The cutoff frequency in Hz
    /// @param zeta: The damping ratio (unitless) - default is
    ///              1 / SQRT(2) for critical damping
    ////////////////////////////////////////////////////////////
    HighPassFilter(float Ts, float fc, float zeta);

    //////////////////////////////////////////////////////////
    /// @brief The high pass filter takes three arguments
    ///        and then calculates the correct coefficients
    ///        This default constructor has hardcoded params
    ///
    ////////////////////////////////////////////////////////////
    HighPassFilter() : HighPassFilter(0.002, 0.01, 1.0/M_SQRT2){};

};


#endif //SRC_HIGHPASSFILTER_H
