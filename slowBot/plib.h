#ifndef PLIB_H
#define PLIB_H

#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <cmath>

#define MAX_POWER 100

#define LOOP_TIME 0.020   // 20 ms, 50 Hz

using namespace std;

// Simple PID class
// kP is constants for PID
class PID {
    public:
        PID(float p);
        void setConstants(float p);
        void initialize();
        float calculate(float target, float sensorValue);
    private:
        float kP;
};

// Simple PID object constructor
// kP set to inputted value
PID::PID(float p) {
    kP = p;
}

// Simple PID function setConstants
// Allows user to input kP
// kP set to inputted value
void PID::setConstants(float p) {
    kP = p;
}

// Simple PID function calculate
// Calculates control loop output
float PID::calculate(float target, float sensorValue) {
    // Declare variables
    float error, output;

    // Calculate error (P)
    error = target - sensorValue;

    // Calculate output
    output = kP * error;

    // Limit output with threshold
    if (fabs(output) > MAX_POWER) {
        output = MAX_POWER * output / fabs(output);
    }

    return output;
}

#endif // PLIB_H
