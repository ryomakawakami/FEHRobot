#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <pidlib.h>

#define MAX_STEP 10  // Max change per iteration
#define LOOP_TIME 0.020   // 20 ms, 50 Hz

#define MAX_VELOCITY 5 // Max (reasonable) velocity in in/s
#define VELOCITY_RANGE 10 // Range of velocity PID in in

#define TICKS_PER_INCH 39 // Conversion from encoder ticks to in

// PID objects with random constants
PID basePID(0.1, 0, 0, 0), velocityPID(0.3, 0, 0, 0), driftPID(0.1, 0, 0, 0);

// Declare servo
FEHServo armServo(FEHServo::Servo0);

// Declare motors
FEHMotor leftBase(FEHMotor::Motor0, 9);
FEHMotor rightBase(FEHMotor::Motor1, 9);

// Declare encoders
DigitalEncoder leftEnc(FEHIO::P0_0);
DigitalEncoder rightEnc(FEHIO::P0_1);

// Sets left base motor
void driveL(int power) {
    leftBase.SetPercent(power);
}

// Sets right base motor
void driveR(int power) {
    rightBase.SetPercent(power);
}

// Gets left encoder
float getLeftEnc() {
    return leftEnc.Counts();
}

// Gets right encoder
float getRightEnc() {
    return rightEnc.Counts();
}

// Clears left encoder
void clearLeftEnc() {
    leftEnc.ResetCounts();
}

// Clears right encoder
void clearRightEnc() {
    rightEnc.ResetCounts();
}

// Conversion from inch to ticks
float inchToTicks(float target) {
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
void autoDrive(float target, float vTarget = MAX_VELOCITY, float vRange = VELOCITY_RANGE) {
    bool done = false;
    float closeTime;
    float driveOut, driftOut;
    float outL, outR, lastOutL = 0, lastOutR = 0;
    float avgEnc, currentTime;
    float velocity, lastAvgEnc = 0, lastTime = TimeNow() - LOOP_TIME;

    target = inchToTicks(target);
    vRange = inchToTicks(vRange) * target / fabs(target);

    basePID.initialize();
    velocityPID.initialize();
    driftPID.initialize();

    // Consider allowing for accumulating error
    clearLeftEnc();
    clearRightEnc();

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

        // Calculate motor outputs
        // Limit driveOut contribution so driftOut can have affect it?
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

// PID control loop without velocity control
// target is desired encoder count
// Position PID when some distance away
// Drift PID and slew rate are constantly active
// Ends function once at location
// MAX_STEP is slew rate limit (10%)
// LOOP_TIME is time per update (20 ms)
void autoDriveIncremental(float target) {
    bool done = false;
    float driveOut, driftOut;
    float outL, outR, lastOutL = 0, lastOutR = 0;
    float avgEnc, currentTime;

    target = inchToTicks(target);

    basePID.initialize();
    driftPID.initialize();

    // Consider allowing for accumulating error
    clearLeftEnc();
    clearRightEnc();

    while(!done) {
        // Update average distance
        avgEnc = (getLeftEnc() + getRightEnc()) / 2;

        // Position PID
        driveOut = basePID.calculate(target, avgEnc);

        // Drift PID
        driftOut = driftPID.calculate(0, getLeftEnc() - getRightEnc());

        // Calculate motor outputs
        // Limit driveOut contribution so driftOut can have affect it?
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

        if(target - avgEnc < 25) {
            done = true;
        }
    }

    // Stop motors
    driveL(0);
    driveR(0);
}

int main(void)
{
    float x, y;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    armServo.TouchCalibrate();

    autoDriveIncremental(12);

    return 0;
}
