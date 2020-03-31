//
// Created by james on 3/29/20.
//

#include <bobble_controllers/PIDFilters.h>

DerivativeFilter::DerivativeFilter(double Ts, double fc, double Kd){
    resetFilterParameters(Ts, fc, Kd);
}

void DerivativeFilter::resetFilterParameters(double Ts, double fc, double Kd)
{
    /// Initialize buffers to 0
    initBuffer(_inputBuffer, 2);
    initBuffer(_outputBuffer, 2);

    _numInWeights = 2;
    _numOutWeights = 2;
    double wc = (2.0 * M_PI * fc);
    /// The following calculations are the implementation of a derivative and first order filter
    /// converted from continuous space to discrete space using the bilinear transform.
    /// The continuous time filter is s Kd wc / (s + wc)
    double numeratorWeights[2] = {2.0*Kd*wc, -2.0*Kd*wc};
    double denominatorWeights[2] = {2.0 + wc*Ts, wc*Ts - 2.0};

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

IntegralFilter::IntegralFilter(double Ts, double Ki){
    resetFilterParameters(Ts, Ki);
}

void IntegralFilter::resetFilterParameters(double Ts, double Ki)
{
    /// Initialize buffers to 0
    initBuffer(_inputBuffer, 2);
    initBuffer(_outputBuffer, 2);

    _numInWeights = 2;
    _numOutWeights = 2;
    /// The following calculations are the implementation of a derivative and first order filter
    /// converted from continuous space to discrete space using the bilinear transform.
    /// The continuous time filter is s Kd wc / (s + wc)
    double numeratorWeights[2] = {Ts*Ki, Ts*Ki};
    double denominatorWeights[2] = {2.0, -2.0};

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

