#ifndef PIDLIB
#define PIDLIB

#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <cmath>

using namespace std;

// PID class
// lastTime stores time of previous calculation
// kP, kI, kD, kF are constants for PID
// lastError stores the last value of the error
// sigma stores the total error
class PID {
    public:
        PID(double p, double i, double d, double f);
        void setConstants(double p, double i, double d, double f);
        double calculate(double target, double sensorValue, double range = 10000);
    private:
        double lastTime;
        double kP, kI, kD, kF;
        double lastError;
        double sigma;
};

// PID object constructor
// lastTime set to current time
// kP, kI, kD, kF set to inputted values
// lastError, sigma set to 0
PID::PID(double p, double i, double d, double f) {
    lastTime = TimeNow();
    kP = p;
    kI = i;
    kD = d;
    kF = f;
    lastError = 0;
    sigma = 0;
}

// PID function setConstants
// Allows user to input kP, kI, kD and initializes other variables
// lastTime set to current time
// kP, kI, kD, kF set to inputted values
// lastError, sigma set to 0
void PID::setConstants(double p, double i, double d, double f) {
    lastTime = TimeNow();
    kP = p;
    kI = i;
    kD = d;
    kF = f;
    lastError = 0;
    sigma = 0;
}

// PID function calculate
// Calculates control loop output
// Integral range default is a large value
double PID::calculate(double target, double sensorValue, double range) {
    // Declare variables
    double deltaTime, error, derivative, output;

    // Find change in time and store current
    double currentTime = TimeNow();
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
    derivative = (error - lastError) / deltaTime;
    lastError = error;

    // Calculate output
    output = kP * error + kI * sigma + kD * derivative + kF * target;

    return output;
}

#endif
