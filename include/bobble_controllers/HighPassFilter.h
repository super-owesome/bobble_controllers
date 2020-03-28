//
// Created by james on 3/27/20.
//

#ifndef SRC_HIGHPASSFILTER_H
#define SRC_HIGHPASSFILTER_H

#include "Filter.h"

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
    HighPassFilter()
    {
        /// 500 Hz sample, 0.01 Hz cutoff, 1/sqrt(2) damping
        HighPassFilter(0.002, 0.01, 0.707);
    };

};


#endif //SRC_HIGHPASSFILTER_H
