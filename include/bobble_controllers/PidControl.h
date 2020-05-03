#ifndef PIDCONTROL_H
#define PIDCONTROL_H

#include <bobble_controllers/PIDFilters.h>

enum PID_Direction : bool {
    NORMAL = true,
    REVERSED = false,
};

class PidControl{
public:
    PidControl(double p, double i, double d, double fc, double sampleTime, bool direction = PID_Direction::NORMAL);
    PidControl(double p, double i, double d, double fc, double sampleTime, double f, bool direction = PID_Direction::NORMAL);
    void setPID(double f, double p, double i, double d, double fc, double sampleTime, bool direction = PID_Direction::NORMAL);
    void setMaxIOutput(double);
    void setOutputLimits(double);
    void setOutputLimits(double,double);
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
    bool direction;
    bool useExternalDerivError;

    double outputRampRate;
    double lastOutput;

    double outputFilter;

    double setpointRange;
};
#endif
