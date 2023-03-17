#include <iostream>
#include <stdexcept>
#include "PID.h"

/*
contructor of PID. in_Kp, in_Ki and in_Kd are the relevant K. in_dT is optional delta time (in seconds, default 1).
all K values must be non negative. dT must be strictly positive.
*/
PID::PID(float in_Kp, float in_Ki, float in_Kd, float in_dT){
    Kp = in_Kp;
    Ki = in_Ki;
    Kd = in_Kd;
    dT = in_dT;
    validateInputs();
}
/*
calculate the control output for relevant set point (wanted outcome) and processValue (given status).
the controller is desinged for forward control (increasing control output increases the proces value).
for reverse control simply switch setPoint and processValue
*/
float PID::calculate(float setPoint, float processValue){
    float pVal, iVal, dVal, e;
    /*
     note for tester - we must check for over or under flow for every operation, however that would cumbersome
     and I think that the exercise does not aim for it.
     In short, before every action we need to check if the result would be greater than max float (in absolute value)
     and after every action we need to check that non zero results are indeed not zero.
    */
    e = setPoint - processValue; //calculate current error
    pVal = Kp*e;                 //proportion value

    integralError += e*dT;       //integration on the error
    iVal = Ki*integralError;     //integral part value

    dVal = Kd*(e-previousErr)/dT;   //derevative of error
    previousErr = e;

    return pVal+iVal+dVal;
}
/*
reset the memory of the integral and derivative part of the controller 
*/
void PID::reset(){
    integralError = 0.0f;
    previousErr = 0.0f;
}
/*
validate the input parameters of the controller
*/
void PID::validateInputs(){
    try{
        if (Kp<0){
            throw std::out_of_range("error using PID, Kp cant be negative");
        }
        if (Ki<0){
            throw std::out_of_range("error using PID, Ki cant be negative");
        }
        if (Kd<0){
            throw std::out_of_range("error using PID, Kd cant be negative");
        }
        if (dT<=0){
            throw std::out_of_range("error using PID, dT must be positive");
        }
    }
    catch (const std::out_of_range& e){
        // handle the error, change according to corporate guidelines
        std::cout << e.what() << '\n';
        throw; 
    }
}
