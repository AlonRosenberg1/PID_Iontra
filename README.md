# PID_Iontra
 exercise of basic PID build in CPP.
 This repository includes the PID controller itself (PID.cpp and h) and a main function which shows how to use the controller.
 It also contain a test vector (SP.txt) for main function to use, where the goal is to generate output vectors to be analysed in your favorit analysis tool (python/matlab etc). 
 Generating a unit test by comparing outputs to a golden known vectors could also have been done.
 
 # theory
 PID controller design to minimize the difference between wanted state (set value) and given output of a system (process value).
 It does so by combining 3 parts - propotion of the current difference between set and process values,
 propotion of all the history of the difference (integral part) and propotion of the current derivative of this difference.
 
 # usage
 PID pid_instance(Kp,Ki,Kd,dt);
 creates instance with given K's. all K values must by non negative.
 dT is optional and must be stricly positive.
 all values are floats
 
 control_output = pid_instance.calculate(setValue, processValue);
 calculates the control output for given set and process values.
 all values are floats.
 
 pid_instance.reset();
 deletes all memory parts (integral and derivative)  of the controler and returns them to zero;
