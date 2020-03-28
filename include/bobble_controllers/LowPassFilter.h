//
// Created by james on 3/28/20.
//

#ifndef SRC_LOWPASSFILTER_H
#define SRC_LOWPASSFILTER_H

#include <bobble_controllers/Filter.h>
#include <math.h>

class LowPassFilter : public Filter
{
public:
    //////////////////////////////////////////////////////////
    /// @brief The low pass filter takes three arguments
    ///        and then calculates the correct coefficients
    ///
    /// @param Ts: The sample period in seconds
    /// @param fc: The cutoff frequency in Hz
    /// @param zeta: The damping ratio (unitless) - default is
    ///              1.0 for no overshoot
    ////////////////////////////////////////////////////////////
    LowPassFilter(double Ts, double fc, double zeta);

    //////////////////////////////////////////////////////////
    /// @brief The low pass filter takes three arguments
    ///        and then calculates the correct coefficients
    ///        This default constructor has hardcoded params
    ///
    ////////////////////////////////////////////////////////////
    LowPassFilter() : LowPassFilter(0.002, 50.0, 1.0){};

    void resetFilterParameters(double Ts, double fc, double zeta);
};
#endif //SRC_LOWPASSFILTER_H
