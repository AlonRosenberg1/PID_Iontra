#ifndef _PID_
#define _PID_

//class which implement PID controller
class PID{
    public:
    // methods decleration
    PID(float in_Kp, float in_Ki, float in_Kd, float in_dT = 1.0f); //c'tor
    float calculate(float setPoint, float processValue); //return desired control output for given SP and PV
    void reset();  // reset memory of integral and derivative parts to zero

    private:
    float Kp, Ki, Kd;        //coefficients of P I and D parts. must be non negative
    float dT;             // time between each call for the controller. seconds
    float integralError = 0.0f; // integration of all previous errors for I part. default is 0
    float previousErr = 0.0f;   // previous error save for D part. default is 0
    void validateInputs();   // validate K values and dT are valid
};

#endif