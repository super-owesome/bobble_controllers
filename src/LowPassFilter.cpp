//
// Created by james on 3/28/20.
//

#include <bobble_controllers/LowPassFilter.h>

LowPassFilter::LowPassFilter(double Ts, double fc, double zeta) {
    resetFilterParameters(Ts, fc, zeta);
}

void LowPassFilter::resetFilterParameters(double Ts, double fc, double zeta) {
    /// Initialize buffers to 0
    initBuffer(_inputBuffer, 3);
    initBuffer(_outputBuffer, 3);

    _numInWeights = 3;
    _numOutWeights = 3;
    double wc = 2.0 * M_PI * fc;
    /// The following calculations are the implementation of a second order high pass filter
    /// converted from continuous space to discrete space using the bilinear transform.
    /// The continuous time filter is 1 / (s^2 + 2*zeta*wc*s + wc^2)
    double numeratorWeights[3] = {Ts*Ts*wc*wc, 2*Ts*Ts*wc*wc, Ts*Ts*wc*wc};
    double denominatorWeights[3] = {Ts*Ts*wc*wc+4*Ts*wc*zeta+4,
                                   2*Ts*Ts*wc*wc-8,
                                   Ts*Ts*wc*wc-4*Ts*wc*zeta+4};

    /// First initialize all weights to 0.0
    for (unsigned int i = 0; i<MAX_FILTER_SIZE; i++){
        _inputWeights[i] = 0.0;
        _outputWeights[i] = 0.0;
    }
    /// Apply the passed in weights.
    for (unsigned int i = 0; i<_numInWeights; i++){
        _inputWeights[i] = numeratorWeights[i];
    }
    for (unsigned int i = 0; i<_numOutWeights; i++){
        _outputWeights[i] = denominatorWeights[i];
    }

}

