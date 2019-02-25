#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHAccel.h>
#include <pidlib.h>

#define MIN_SPEED 10

#define MAX_STEP 10  // Max change per iteration
#define LOOP_TIME 0.020   // 20 ms, 50 Hz

#define TICKS_PER_INCH 28 // Conversion from encoder ticks to in

#define straightP 0.5
#define straightI 0.001

#define driftP 2

#define turnP 0.5
#define turnI 0.001

#define RED_THRESHOLD 0.5 // Threshold for red light vs no light

// Declare motors
FEHMotor leftBase(FEHMotor::Motor0, 9);
FEHMotor rightBase(FEHMotor::Motor1, 9);

// Declare encoders
DigitalEncoder leftEnc(FEHIO::P0_1);
DigitalEncoder rightEnc(FEHIO::P1_0);

// PID control loop
// target is desired encoder count
// Position PID when some distance away
// Drift PID and slew rate are constantly active
// Ends function 250 ms after gets close
// MAX_STEP is slew rate limit (10%)
// LOOP_TIME is time per update (20 ms)
void autoDrive(float target) {
    PID basePID(straightP, straightI, 0, 0), driftPID(driftP, 0, 0, 0);

    bool done = false;
    float closeTime;
    float driveOut, driftOut;
    float outL, outR, lastOutL = 0, lastOutR = 0;
    float avgEnc, currentTime;

    target *= TICKS_PER_INCH;

    basePID.initialize();
    driftPID.initialize();

    // Consider allowing for accumulating error
    leftEnc.ResetCounts();
    rightEnc.ResetCounts();

    while(!done) {
        // Update current time
        currentTime = TimeNow();

        // Update average distance
        avgEnc = (leftEnc.Counts() + rightEnc.Counts()) / 2;

        // Position PID
        driveOut = basePID.calculate(target, avgEnc);

        // Drift PID
        driftOut = driftPID.calculate(0, leftEnc.Counts() - rightEnc.Counts());

        // Calculate motor outputs
        // Limit driveOut contribution so driftOut can have affect it?
        outL = driveOut;
        outR = driveOut;

        // Slew rate limit
        if(outL + driftOut - lastOutL > MAX_STEP) {
            outL = lastOutL + 10;
        }
        else if(outL + driftOut - lastOutL < -MAX_STEP) {
            outL = lastOutL - 10;
        }

        if(outR - driftOut - lastOutR > MAX_STEP) {
            outR = lastOutR + 10;
        }
        else if(outR - driftOut - lastOutR < -MAX_STEP) {
            outR = lastOutR - 10;
        }

        // Set motors to output
        leftBase.SetPercent(outL + driftOut);
        rightBase.SetPercent(outR - driftOut);

        // Store output for slew rate
        lastOutL = outL + driftOut;
        lastOutR = outR - driftOut;

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
    leftBase.SetPercent(0);
    rightBase.SetPercent(0);
}

// PID control loop
// target is desired encoder count
// Position PID when some distance away
// Drift PID and slew rate are constantly active
// Ends function 250 ms after gets close
// MAX_STEP is slew rate limit (10%)
// LOOP_TIME is time per update (20 ms)
void autoTurn(float target) {
    PID turnPID(turnP, turnI, 0, 0), driftPID(driftP, 0, 0, 0);

    bool done = false;
    float closeTime;
    float driveOut, driftOut;
    float outL, outR, lastOutL = 0, lastOutR = 0;
    float avgEnc, currentTime;

    target *= TICKS_PER_INCH;

    turnPID.initialize();
    driftPID.initialize();

    // Consider allowing for accumulating error
    leftEnc.ResetCounts();
    rightEnc.ResetCounts();

    while(!done) {
        // Update current time
        currentTime = TimeNow();

        // Update average distance
        avgEnc = (leftEnc.Counts() + rightEnc.Counts()) / 2;

        // Position PID
        driveOut = turnPID.calculate(target, avgEnc);

        // Drift PID
        driftOut = driftPID.calculate(0, leftEnc.Counts() - rightEnc.Counts());

        // Calculate motor outputs
        // Limit driveOut contribution so driftOut can have affect it?
        outL = driveOut;
        outR = driveOut;

        // Slew rate limit
        if(outL + driftOut - lastOutL > MAX_STEP) {
            outL = lastOutL + 10;
        }
        else if(outL + driftOut - lastOutL < -MAX_STEP) {
            outL = lastOutL - 10;
        }

        if(outR - driftOut - lastOutR > MAX_STEP) {
            outR = lastOutR + 10;
        }
        else if(outR - driftOut - lastOutR < -MAX_STEP) {
            outR = lastOutR - 10;
        }

        // Set motors to output
        leftBase.SetPercent(outL + driftOut);
        rightBase.SetPercent(outR - driftOut);

        // Store output for slew rate
        lastOutL = outL + driftOut;
        lastOutR = outR - driftOut;

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
    leftBase.SetPercent(0);
    rightBase.SetPercent(0);
}

void setBase(int power) {
    leftBase.SetPercent(power);
    rightBase.SetPercent(-power);
}

void timeDrive(int power, int time) {
    setBase(power);
    Sleep(time);
    setBase(0);
}

void upRamp() {
    setBase(30);
    while(Accel.Y() < 0.25) {
        LCD.WriteLine(Accel.Y());
        Sleep(100);
    }
    setBase(50);
    while(Accel.Y() > 0.25);
    Sleep(500);
    setBase(0);
}

int main(void)
{
    float x, y;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    return 0;
}
