#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHAccel.h>
#include "pidlib.h"

#define MIN_SPEED 8

#define MAX_STEP 10  // Max change per iteration
#define LOOP_TIME 0.020   // 20 ms, 50 Hz

#define TICKS_PER_INCH 28 // Conversion from encoder ticks to in

enum {
    RUN,
    ADJUST
};

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
// Ends function once at location
// MAX_STEP is slew rate limit (10%)
// LOOP_TIME is time per update (20 ms)
void autoDriveF(float target, float kP) {
    PID basePID(kP, 0.01, 0, 0), driftPID(2, 0, 0, 0);

    bool done = false;
    float driveOut, driftOut;
    float outL, outR, lastOutL = 0, lastOutR = 0;
    float avgEnc;

    target *= TICKS_PER_INCH;

    basePID.initialize();
    driftPID.initialize();

    // Consider allowing for accumulating error
    leftEnc.ResetCounts();
    rightEnc.ResetCounts();

    while(!done) {
        // Update average distance
        avgEnc = rightEnc.Counts();

        // Position PID
        driveOut = basePID.calculate(target, avgEnc);

        // Drift PID
        driftOut = driftPID.calculate(0, leftEnc.Counts() - rightEnc.Counts());

        // Calculate motor outputs
        // Limit driveOut contribution so driftOut can have affect it?
        outL = driveOut + driftOut;
        outR = driveOut - driftOut;

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

        // Make sure output is at least minimum speed
        if(fabs(outL) < MIN_SPEED) {
            outL = MIN_SPEED * outL / fabs(outL);
        }
        if(fabs(outR) < MIN_SPEED) {
            outR = MIN_SPEED * outR / fabs(outR);
        }

        // Set motors to output
        leftBase.SetPercent(outL);
        rightBase.SetPercent(-outR);

        // Store output for slew rate
        lastOutL = outL;
        lastOutR = outR;

        // Sleep for set time
        Sleep(LOOP_TIME);

        if(target - avgEnc < 0) {
            done = true;
        }
    }

    // Stop motors
    leftBase.SetPercent(0);
    rightBase.SetPercent(0);
}

int main(void)
{

    float x,y;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    float kP = 0;

    bool statusDisplayed = false;
    int status = 0;

    while(true)
    {
        if(Accel.Y() > -0.3) {
            if(status != RUN) {
                statusDisplayed = false;
                status = RUN;
            }
        }
        else {
            if(status != ADJUST) {
                statusDisplayed = false;
                status = ADJUST;
            }
        }

        if(!statusDisplayed) {
            switch(status) {
            case RUN:
                LCD.Clear(FEHLCD::Black);
                LCD.WriteLine("Touch to run");
            break;
            case ADJUST:
                LCD.Clear(FEHLCD::Black);
                LCD.WriteLine("Adjusting kP");
                LCD.WriteRC(kP, 1, 0);
            break;
            }
            statusDisplayed = true;
        }

        while(LCD.Touch(&x, &y)) {
            switch(status) {
            case RUN:
                autoDriveF(12, kP);
            break;
            case ADJUST:
                if(x > 160) {
                    kP += 0.1;
                    LCD.WriteRC(kP, 1, 0);
                    while(LCD.Touch(&x, &y));
                    Sleep(100);
                }
                else {
                    kP -= 0.1;
                    if(kP < 0) {
                        kP = 0;
                    }
                    LCD.WriteRC(kP, 1, 0);
                    while(LCD.Touch(&x, &y));
                    Sleep(100);
                }
            break;
            }
        }
        Sleep(50);
    }
    return 0;
}
