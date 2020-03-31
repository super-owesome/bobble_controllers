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
    ///              1.0 for no overshoot
    ////////////////////////////////////////////////////////////
    HighPassFilter(double Ts, double fc, double zeta);

    //////////////////////////////////////////////////////////
    /// @brief The high pass filter takes three arguments
    ///        and then calculates the correct coefficients
    ///        This default constructor has hardcoded params
    ///
    ////////////////////////////////////////////////////////////
    HighPassFilter() : HighPassFilter(0.002, 50, 1.0){};

    void resetFilterParameters(double Ts, double fc, double zeta);
};


#endif //SRC_HIGHPASSFILTER_H
