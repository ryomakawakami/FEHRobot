#ifndef PIDLIB
#define PIDLIB

#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <cmath>

#define MAX_POWER 100

using namespace std;

// PID class
// lastTime stores time of previous calculation
// kP, kI, kD, kF are constants for PID
// lastValue stores the last value of the signal input
// sigma stores the total error
class PID {
    public:
        PID(float p, float i, float d, float f);
        void setConstants(float p, float i, float d, float f);
        float calculate(float target, float sensorValue, float range = 10000);
    private:
        float lastTime;
        float kP, kI, kD, kF;
        float lastValue;
        float sigma;
};

// PID object constructor
// lastTime set to current time
// kP, kI, kD, kF set to inputted values
// lastValue, sigma set to 0
PID::PID(float p, float i, float d, float f) {
    lastTime = TimeNow();
    kP = p;
    kI = i;
    kD = d;
    kF = f;
    lastValue = 0;
    sigma = 0;
}

// PID function setConstants
// Allows user to input kP, kI, kD and initializes other variables
// lastTime set to current time
// kP, kI, kD, kF set to inputted values
// lastValue, sigma set to 0
void PID::setConstants(float p, float i, float d, float f) {
    lastTime = TimeNow();
    kP = p;
    kI = i;
    kD = d;
    kF = f;
    lastValue = 0;
    sigma = 0;
}

// PID function calculate
// Calculates control loop output
// Integral range default is a large value
float PID::calculate(float target, float sensorValue, float range) {
    // Declare variables
    float deltaTime, error, derivative, output;

    // Find change in time and store current
    float currentTime = TimeNow();
    deltaTime = currentTime - lastTime;
    lastTime = currentTime;

    // Calculate error (P)
    error = target - sensorValue;

    // Calculate sigma (I)
    sigma += error * deltaTime;

    // Reset sigma if outside of integral range
    if(fabs(error) > range) {
        // sigma = 0;
    }

    // Also reset if robot shoots over
    else if(target > 0) {
        if(error < 0) {
            // sigma = 0;
        }
    }
    else {
        if(error > 0) {
            // sigma = 0;
        }
    }

    // Calculate derivative (D)
    // Change in value over change in time and store current
    derivative = (sensorValue - lastValue) / deltaTime;
    lastValue = sensorValue;

    // Calculate output
    output = kP * error + kI * sigma + kD * derivative + kF * target;

    // Limit output with threshold
    if (fabs(output) > 100) {
        output = 100 * output / fabs(output);
    }

    return output;
}

#endif
