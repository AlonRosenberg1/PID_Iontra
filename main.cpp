//testing PID controller
#include <iostream>
#include <fstream>

#include "PID.h"

using namespace std;
/*
will test one PID controller (set of K's and dT) with a given input vector (given in txt file)
and will write to a given txt file (for easy analysis in python/matlab)
*/
void test_controller(float Kp, float Ki, float Kd, float dT, string inputFileName, string outputFileName);

// same as test_controller, but will reset the PID after resetTime cycles
void test_controller_reset(float Kp, float Ki, float Kd, float dT, int resetTime, string inputFileName, string outputFileName);

int main(){
    // test propotional part alone
    test_controller(1.0f,0.0f,0.0f,1.0f,"SP.txt","PV_P_alone.txt");
    // test propotional and integral part 
    test_controller(0.5f,0.3f,0.0f,1.0f,"SP.txt","PV_PI.txt");
    // test propotional and derivative part 
    test_controller(0.5f,0.0f,0.2f,1.0f,"SP.txt","PV_PD.txt");
    // test delta time different than 1 part 
    test_controller(0.5f,0.3f,0.0f,2.0f,"SP.txt","PV_dT.txt");

    test_controller_reset(0.5f,0.3f,0.1f,1.0f,9,"SP.txt","PV_reset9.txt");
    // test negative K - expected to crush 
    try{
        test_controller(-0.5f,0.3f,0.0f,1.0f,"SP.txt","PV_negK.txt");
    }
    catch(const std::out_of_range& e){
        cout<<"detected negative K correctly"<<endl;
        cout<<e.what()<<endl;
    }
    return 1;
}

void test_controller(float Kp, float Ki, float Kd, float dT, string inputFileName, string outputFileName){
    float currProcessValue = 0.0f; //initial process value, may be moved to arguments for more usability.
    float setPoint, controlOutput;
    string line;

    PID pidInst(Kp,Ki,Kd,dT);

    ifstream inFile(inputFileName);
    ofstream outFile(outputFileName);

    while(getline(inFile,line)){
        setPoint = stof(line);  //get new wanted set point from file
        controlOutput = pidInst.calculate(setPoint,currProcessValue); // calculate control
        
        /*
        normally we would have our process model calculate new process value given the new control
        here, however we are not given a process model so we assume the process value is the control directly.
        if a process is given this line should change to  currProcessValue = processModle.calculate(controlOutput)
        */
        currProcessValue = controlOutput; 
        
        line = to_string(currProcessValue);
        outFile << line << endl;   // write output to file. in real systems its worthwhile to log both the control and process values
    }
    inFile.close();
    outFile.close();
}

void test_controller_reset(float Kp, float Ki, float Kd, float dT, int resetTime, string inputFileName, string outputFileName){
    float currProcessValue = 0.0f; 
    float setPoint, controlOutput;
    string line;
    int cycles = 1;

    PID pidInst(Kp,Ki,Kd,dT);

    ifstream inFile(inputFileName);
    ofstream outFile(outputFileName);

    while(getline(inFile,line)){
        if(resetTime==cycles){
            pidInst.reset();
        }
        setPoint = stof(line);  
        controlOutput = pidInst.calculate(setPoint,currProcessValue); 
        
        currProcessValue = controlOutput; 
        
        line = to_string(currProcessValue);
        outFile << line << endl;   

        cycles++;
    }
    inFile.close();
    outFile.close();
}