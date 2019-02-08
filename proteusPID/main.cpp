#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHAccel.h>
#include <FEHMotor.h>
#include <pidlib.h>
#include <trueSpeed.h>

#define MAX_STEP 10  // Max change per iteration
#define LOOP_TIME 20   // 20 ms, 50 Hz

#define MAX_VELOCITY 5 // Max (reasonable) velocity in in/s
#define VELOCITY_RANGE 10 // Range of velocity PID in in

#define TICKS_PER_INCH 50 // Conversion from encoder ticks to in

// PID objects with random constants
PID basePID(0.5, 0, 0, 0), velocityPID(1, 0, 0, 0), driftPID(1, 0, 0, 0);

// Declare motors
FEHMotor leftBase(FEHMotor::Motor0, 9);
FEHMotor rightBase(FEHMotor::Motor1, 9);

// Declare encoders
AnalogInputPin leftEnc(FEHIO::P0_0);
AnalogInputPin rightEnc(FEHIO::P0_1);

// Sets left base motor
void driveL(int power) {
    leftBase.SetPercent(power);
}

// Sets right base motor
void driveR(int power) {
    rightBase.SetPercent(power);
}

// Gets left encoder
double getLeftEnc() {
    return leftEnc.Value();
}

// Gets right encoder
double getRightEnc() {
    return rightEnc.Value();
}

// Conversion from inch to ticks
double inchToTicks(double target) {
    return target * TICKS_PER_INCH;
}

// PID control loop
// target is desired encoder count, vTarget is desired velocity, vRange is active range of velocity PID
// Velocity PID until some distance to target
// Position PID when some distance away
// Drift PID and slew rate are constantly active
// Ends function 250 ms after gets close
// MAX_STEP is slew rate limit (10%)
// LOOP_TIME is time per update (20 ms)
void autoDrive(double target, double vTarget = MAX_VELOCITY, double vRange = VELOCITY_RANGE) {
    bool done = false;
    double closeTime;
    double driveOut, driftOut;
    double outL, outR, lastOutL = 0, lastOutR = 0;
    double avgEnc, currentTime;
    double velocity, lastAvgEnc = 0, lastTime = TimeNow() - LOOP_TIME;

    target = inchToTicks(target);
    vRange = inchToTicks(vRange) * target / fabs(target);

    while(!done) {
        // Update current time
        currentTime = TimeNow();

        // Update average distance
        avgEnc = (getLeftEnc() + getRightEnc()) / 2;

        // Calculate velocity
        velocity = (avgEnc - lastAvgEnc) / (currentTime - lastTime);
        lastAvgEnc = avgEnc;
        lastTime = currentTime;

        // Velocity PID
        if(fabs(target) - fabs(avgEnc) > vRange) {
            driveOut = velocityPID.calculate(vTarget, velocity);
        }

        // Position PID
        else {
            driveOut = basePID.calculate(target, avgEnc);
        }

        // Drift PID
        driftOut = driftPID.calculate(0, getLeftEnc() - getRightEnc());

        // Calculate mtoor outputs
        outL = driveOut - driftOut;
        outR = driveOut + driftOut;

        // Slew rate limit
        if(outL - lastOutL > MAX_STEP) {
            outL = lastOutL + 10;
        }
        else if(outL - lastOutL < -MAX_STEP) {
            outL = lastOutL - 10;
        }

        if(outR - lastOutR > MAX_STEP) {
            outR = lastOutR + 10;
        }
        else if(outR - lastOutR < -MAX_STEP) {
            outR = lastOutR - 10;
        }

        // Set motors to output
        driveL(outL);
        driveR(outR);

        // Store output for slew rate
        lastOutL = outL;
        lastOutR = outR;

        // Sleep for set time
        Sleep(LOOP_TIME);

        // Loop ends .25 seconds after average gets within 100 ticks
        if(fabs(target - avgEnc) > 100) {
            closeTime = currentTime;
        }

        if(currentTime - closeTime > 250) {
            done = true;
        }
    }

    // Stop motors
    driveL(0);
    driveR(0);
}

int main(void)
{
    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);



    return 0;
}
