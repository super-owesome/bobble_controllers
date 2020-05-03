/**
* Small, easy to use PID implementation. <br>
* Minimal usage:<br>
* setPID(p,i,d); <br>
* ...looping code...{ <br>
* output=getOutput(sensorvalue,target); <br>
* }
*
* @see http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/improving-the-beginners-pid-introduction
*/

#include <bobble_controllers/PidControl.h>

//**********************************
//Constructor functions
//**********************************
PidControl::PidControl(double p, double i, double d, double fc, double sampleTime, bool direction){
    init();
    this->P = p;
    this->I = i;
    this->D = d;
    this->Fc = fc;
    this->Ts = sampleTime;
    this->direction = direction;
    derivativeFilter.resetFilterParameters(Ts, Fc, D);
    integralFilter.resetFilterParameters(Ts, I);
}
PidControl::PidControl(double p, double i, double d, double fc, double sampleTime, double f, bool direction){
    init();
    this->F = f;
    this->P = p;
    this->I = i;
    this->D = d;
    this->Fc = fc;
    this->Ts = sampleTime;
    this->direction = direction;
    derivativeFilter.resetFilterParameters(Ts, Fc, D);
    integralFilter.resetFilterParameters(Ts, I);
}
void PidControl::init(){
    this->P = 0;
    this->I = 0;
    this->D = 0;
    this->F = 0;

    maxIOutput = 0;
    maxError = 0;
    errorSum = 0;
    maxOutput = 0;
    minOutput = 0;
    setpoint = 0;
    lastActual = 0;
    firstRun = true;
    direction = PID_Direction::NORMAL;
    outputRampRate = 0;
    lastOutput = 0;
    outputFilter = 0;
    setpointRange = 0;
    extDerivError = 0;
    useExternalDerivError = false;
}

//**********************************
//Configuration functions
//**********************************
/** Create a new PID object.
 * @param p Proportional gain. Large if large difference between setpoint and target.
 * @param i Integral gain.	Becomes large if setpoint cannot reach target quickly.
 * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents P and I terms from causing overshoot.
 */
void PidControl::setPID(double f, double p, double i, double d, double fc, double sampleTime, bool direction){
    this->F = f;
    this->P = p;
    this->I = i;
    this->D = d;
    this->Fc = fc;
    this->Ts = sampleTime;
    this->direction = direction;
    derivativeFilter.resetFilterParameters(Ts, Fc, D);
    integralFilter.resetFilterParameters(Ts, I);
}

void PidControl::setExternalDerivativeError(double* err_source){
    if (err_source){
        extDerivError = err_source;
        useExternalDerivError = true;
    }
}

/**Set the maximum output value contributed by the I component of the system
 * this->can be used to prevent large windup issues and make tuning simpler
 * @param maximum. Units are the same as the expected output value
 */
void PidControl::setMaxIOutput(double maximum){
    /* Internally maxError and Izone are similar, but scaled for different purposes.
     * The maxError is generated for simplifying math, since calculations against
     * the max error are far more common than changing the I term or Izone.
     */
    maxIOutput=maximum;
    if(I!=0){
        maxError=maxIOutput/I;
    }
}

/**Specify a maximum output. If a single parameter is specified, the minimum is
 * set to (-maximum).
 * @param output
 */
void PidControl::setOutputLimits(double output){
    setOutputLimits(-output,output);
}

/**
 * Specify a maximum output.
 * @param minimum possible output value
 * @param maximum possible output value
 */
void PidControl::setOutputLimits(double minimum,double maximum){
    if(maximum<minimum)return;
    maxOutput=maximum;
    minOutput=minimum;

    // Ensure the bounds of the I term are within the bounds of the allowable output swing
    if(maxIOutput==0 || maxIOutput>(maximum-minimum) ){
        setMaxIOutput(maximum-minimum);
    }
}

//**********************************
//Primary operating functions
//**********************************

/** Calculate the PID value needed to hit the target setpoint.
* Automatically re-calculates the output at each call.
* @param actual The monitored value
* @param target The target value
* @return calculated output value for driving the actual to the target
*/
double PidControl::getOutput(double desired, double actual){
    double output;
    double Poutput;
    double Ioutput;
    double Doutput;
    double Foutput;

    this->setpoint = desired;

    //Ramp the setpoint used for calculations if user has opted to do so
    if(setpointRange!=0){
        setpoint=clamp(setpoint,actual-setpointRange,actual+setpointRange);
    }

    //Do the simple parts of the calculations
    double error = setpoint - actual;

    //Calculate F output. Notice, this->depends only on the setpoint, and not the error.
    Foutput = F * setpoint;

    //Calculate P term
    Poutput = P * error;

    //If this->is our first time running this-> we don't actually _have_ a previous input or output.
    //For sensor, sanely assume it was exactly where it is now.
    //For last output, we can assume it's the current time-independent outputs.
    if(firstRun){
        lastActual = actual;
        lastOutput = Poutput + Foutput;
        firstRun = false;
    }

    //Calculate D Term
    // If using the external derivative, use the negative value of Kd
    // If using the standard PID, the derivative term is calcluated
    // from the bilinear transform with a first order filter
    if (useExternalDerivError){
        Doutput= -D * (*extDerivError);
    }else{
        Doutput = derivativeFilter.filter(error);
    }



    //The Iterm is more complex. There's several things to factor in to make it easier to deal with.
    // 1. maxIoutput restricts the amount of output contributed by the Iterm.
    // 2. prevent windup by not increasing errorSum if we're already running against our max Ioutput
    // 3. prevent windup by not increasing errorSum if output is output=maxOutput
    Ioutput = integralFilter.filter(error);
    if(maxIOutput!=0){
        Ioutput = clamp(Ioutput, -maxIOutput, maxIOutput);
    }

    //And, finally, we can just add the terms up
    output = Foutput + Poutput + Ioutput + Doutput;

    //Figure out what we're doing with the error.
    if(minOutput!=maxOutput && !bounded(output, minOutput,maxOutput) ){
        errorSum=error;
        // reset the error sum to a sane level
        // Setting to current error ensures a smooth transition when the P term
        // decreases enough for the I term to start acting upon the controller
        // From that point the I term will build up as would be expected
    }
    else if(outputRampRate!=0 && !bounded(output, lastOutput-outputRampRate,lastOutput+outputRampRate) ){
        errorSum=error;
    }
    else if(maxIOutput!=0){
        errorSum=clamp(errorSum+error,-maxError,maxError);
        // In addition to output limiting directly, we also want to prevent I term
        // buildup, so restrict the error directly
    }
    else{
        errorSum+=error;
    }

    //Restrict output to our specified output and ramp limits
    if(outputRampRate!=0){
        output=clamp(output, lastOutput-outputRampRate,lastOutput+outputRampRate);
    }
    if(minOutput!=maxOutput){
        output=clamp(output, minOutput,maxOutput);
    }
    if(outputFilter!=0.0){
        output=lastOutput*outputFilter+output*(1.0-outputFilter);
    }

    lastOutput=output;

    // If the directionality of the PID is reversed, negate the output
    if(this->direction == PID_Direction::REVERSED)
    {
        output = -output;
    }
    return output;
}

/**
 * Calculates the PID value using the last provided setpoint and actual valuess
 * @return calculated output value for driving the actual to the target
 */
double PidControl::getOutput(){
    return getOutput(lastActual,setpoint);
}

/**
 *
 * @param actual
 * @return calculated output value for driving the actual to the target
 */
double PidControl::getOutput(double actual){
    return getOutput(actual,setpoint);
}

/**
 * Resets the controller. this->erases the I term buildup, and removes D gain on the next loop.
 */
void PidControl::reset(){
    firstRun=true;
    errorSum=0;
}

/**Set the maximum rate the output can increase per cycle.
 * @param rate
 */
void PidControl::setOutputRampRate(double rate){
    outputRampRate=rate;
}

/** Set a limit on how far the setpoint can be from the current position
 * <br>Can simplify tuning by helping tuning over a small range applies to a much larger range.
 * <br>this->limits the reactivity of P term, and restricts impact of large D term
 * during large setpoint adjustments. Increases lag and I term if range is too small.
 * @param range
 */
void PidControl::setSetpointRange(double range){
    setpointRange=range;
}

/**Set a filter on the output to reduce sharp oscillations. <br>
 * 0.1 is likely a sane starting value. Larger values P and D oscillations, but force larger I values.
 * Uses an exponential rolling sum filter, according to a simple <br>
 * <pre>output*(1-strength)*sum(0..n){output*strength^n}</pre>
 * @param output valid between [0..1), meaning [current output only.. historical output only)
 */
void PidControl::setOutputFilter(double strength){
    if(strength==0 || bounded(strength,0,1)){
        outputFilter=strength;
    }
}

//**************************************
// Helper functions
//**************************************

/**
 * Forces a value into a specific range
 * @param value input value
 * @param min maximum returned value
 * @param max minimum value in range
 * @return Value if it's within provided range, min or max otherwise
 */
double PidControl::clamp(double value, double min, double max){
    if(value > max){ return max;}
    if(value < min){ return min;}
    return value;
}

/**
 * Test if the value is within the min and max, inclusive
 * @param value to test
 * @param min Minimum value of range
 * @param max Maximum value of range
 * @return
 */
bool PidControl::bounded(double value, double min, double max){
    return (min<value) && (value<max);
}

