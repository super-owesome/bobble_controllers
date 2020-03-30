#ifndef PIDCONTROL_H
#define PIDCONTROL_H

#include <bobble_controllers/PIDFilters.h>

class PidControl{
public:
    PidControl(double p, double i, double d, double fc, double sampleTime);
    PidControl(double p, double i, double d, double fc, double sampleTime, double f);
    void setP(double p);
    void setI(double i);
    void setD(double d, double fc);
    void setF(double f);
    void setPID(double p, double i, double d, double fc, double sampleTime);
    void setPID(double p, double i, double d, double fc, double sampleTime, double f);
    void setMaxIOutput(double);
    void setOutputLimits(double);
    void setOutputLimits(double,double);
    void setDirection(bool);
    void setSetpoint(double);
    void reset();
    void setOutputRampRate(double);
    void setExternalDerivativeError(double*);
    void setSetpointRange(double);
    void setOutputFilter(double);
    double getOutput();
    double getOutput(double);
    double getOutput(double, double);

private:
    double clamp(double, double, double);
    bool bounded(double, double, double);
    void checkSigns();
    void init();
    double P;
    double I;
    double D;
    double Fc;
    double F;
    double Ts;

    DerivativeFilter derivativeFilter;
    IntegralFilter integralFilter;

    double maxIOutput;
    double maxError;
    double errorSum;
    double* extDerivError;

    double maxOutput;
    double minOutput;

    double setpoint;

    double lastActual;

    bool firstRun;
    bool reversed;
    bool useExternalDerivError;

    double outputRampRate;
    double lastOutput;

    double outputFilter;

    double setpointRange;
};
#endif
