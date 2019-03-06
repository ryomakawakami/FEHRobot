#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHAccel.h>
#include "pidlib.h"

#define MIN_SPEED 8
#define MIN_SPEED_TURNING 12
#define MIN_SPEED_SWEEP 12

#define MAX_STEP 7  // Max change per iteration
#define LOOP_TIME 0.020   // 20 ms, 50 Hz

#define TICKS_PER_INCH 28 // Conversion from encoder ticks to in

enum {
    RUN,
    ADJUST,
    DISTANCE,
    SETTING
};

enum {
    AUTO_DRIVE,
    AUTO_TURN,
    AUTO_SWEEP,
    TOKEN
};

// Declare motors
FEHMotor leftBase(FEHMotor::Motor0, 9);
FEHMotor rightBase(FEHMotor::Motor1, 9);

// Declare encoders
DigitalEncoder rightEnc(FEHIO::P0_0);
DigitalEncoder leftEnc(FEHIO::P1_0);

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
            outL = lastOutL + MAX_STEP;
        }
        else if(outL - lastOutL < -MAX_STEP) {
            outL = lastOutL - MAX_STEP;
        }

        if(outR - lastOutR > MAX_STEP) {
            outR = lastOutR + MAX_STEP;
        }
        else if(outR - lastOutR < -MAX_STEP) {
            outR = lastOutR - MAX_STEP;
        }

        // Make sure output is at least minimum speed
        if(fabs(outL) < MIN_SPEED) {
            outL = MIN_SPEED * outL / fabs(outL);
        }
        if(fabs(outR) < MIN_SPEED) {
            outR = MIN_SPEED * outR / fabs(outR);
        }

        // Set motors to output
        leftBase.SetPercent(-outL);
        rightBase.SetPercent(outR);

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

void autoDriveB(float target, float kP) {
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
            outL = lastOutL + MAX_STEP;
        }
        else if(outL - lastOutL < -MAX_STEP) {
            outL = lastOutL - MAX_STEP;
        }

        if(outR - lastOutR > MAX_STEP) {
            outR = lastOutR + MAX_STEP;
        }
        else if(outR - lastOutR < -MAX_STEP) {
            outR = lastOutR - MAX_STEP;
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

void autoTurnL(float target, float kP) {
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
            outL = lastOutL + MAX_STEP;
        }
        else if(outL - lastOutL < -MAX_STEP) {
            outL = lastOutL - MAX_STEP;
        }

        if(outR - lastOutR > MAX_STEP) {
            outR = lastOutR + MAX_STEP;
        }
        else if(outR - lastOutR < -MAX_STEP) {
            outR = lastOutR - MAX_STEP;
        }

        // Make sure output is at least minimum speed
        if(fabs(outL) < MIN_SPEED_TURNING) {
            outL = MIN_SPEED_TURNING * outL / fabs(outL);
        }
        if(fabs(outR) < MIN_SPEED_TURNING) {
            outR = MIN_SPEED_TURNING * outR / fabs(outR);
        }

        // Set motors to output
        leftBase.SetPercent(outL);
        rightBase.SetPercent(outR);

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

void autoTurnR(float target, float kP) {
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
            outL = lastOutL + MAX_STEP;
        }
        else if(outL - lastOutL < -MAX_STEP) {
            outL = lastOutL - MAX_STEP;
        }

        if(outR - lastOutR > MAX_STEP) {
            outR = lastOutR + MAX_STEP;
        }
        else if(outR - lastOutR < -MAX_STEP) {
            outR = lastOutR - MAX_STEP;
        }

        // Make sure output is at least minimum speed
        if(fabs(outL) < MIN_SPEED_TURNING) {
            outL = MIN_SPEED_TURNING * outL / fabs(outL);
        }
        if(fabs(outR) < MIN_SPEED_TURNING) {
            outR = MIN_SPEED_TURNING * outR / fabs(outR);
        }

        // Set motors to output
        leftBase.SetPercent(-outL);
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

void autoSweepL(float target, float kP) {
    PID basePID(kP, 0.01, 0, 0);

    bool done = false;
    float out, lastOut = 0;
    float counts;

    target *= TICKS_PER_INCH;

    basePID.initialize();

    // Consider allowing for accumulating error
    leftEnc.ResetCounts();

    while(!done) {
        // Update average distance
        counts = leftEnc.Counts();

        // Position PID
        out = basePID.calculate(target, counts);

        // Slew rate limit
        if(out - lastOut > MAX_STEP) {
            out = lastOut + MAX_STEP;
        }
        else if(out - lastOut < -MAX_STEP) {
            out = lastOut - MAX_STEP;
        }

        // Make sure output is at least minimum speed
        if(fabs(out) < MIN_SPEED_SWEEP) {
            out = MIN_SPEED_SWEEP * out / fabs(out);
        }

        // Set motors to output
        leftBase.SetPercent(-out);

        // Store output for slew rate
        lastOut = out;

        // Sleep for set time
        Sleep(LOOP_TIME);

        if(target - counts < 0) {
            done = true;
        }
    }

    // Stop motors
    leftBase.SetPercent(0);
    rightBase.SetPercent(0);
}

void autoSweepR(float target, float kP) {
    PID basePID(kP, 0.01, 0, 0);

    bool done = false;
    float out, lastOut = 0;
    float counts;

    target *= TICKS_PER_INCH;

    basePID.initialize();

    // Consider allowing for accumulating error
    rightEnc.ResetCounts();

    while(!done) {
        // Update average distance
        counts = rightEnc.Counts();

        // Position PID
        out = basePID.calculate(target, counts);

        // Slew rate limit
        if(out - lastOut > MAX_STEP) {
            out = lastOut + MAX_STEP;
        }
        else if(out - lastOut < -MAX_STEP) {
            out = lastOut - MAX_STEP;
        }

        // Make sure output is at least minimum speed
        if(fabs(out) < MIN_SPEED_SWEEP) {
            out = MIN_SPEED_SWEEP * out / fabs(out);
        }

        // Set motors to output
        rightBase.SetPercent(out);

        // Store output for slew rate
        lastOut = out;

        // Sleep for set time
        Sleep(LOOP_TIME);

        if(target - counts < 0) {
            done = true;
        }
    }

    // Stop motors
    leftBase.SetPercent(0);
    rightBase.SetPercent(0);
}

void setBase(int power) {
    leftBase.SetPercent(-power);
    rightBase.SetPercent(power);
}

void timeDrive(int power, int time) {
    setBase(power);
    Sleep(time);
    setBase(0);
}

void moveToToken(float kP) {
    PID basePID(kP, 0.01, 0, 0), driftPID(2, 0, 0, 0);

    bool leftDone = false, rightDone = false, done = false;
    float leftOut, rightOut, driveOut, driftOut;
    float outL, outR, lastOutL = 0, lastOutR = 0;
    float avgEnc;

    float rightTarget = 4.5 * TICKS_PER_INCH;
    float leftTarget = rightTarget + 7 * TICKS_PER_INCH;
    float target = 12 * TICKS_PER_INCH;

    // Consider allowing for accumulating error
    leftEnc.ResetCounts();
    rightEnc.ResetCounts();

    while(!done) {
        if(!leftDone) {
            leftOut = 50;
        }
        if(!rightDone) {
            rightOut = 50;
        }
        else {
            rightOut = 0;
        }

        // Slew rate limit
        if(leftOut - lastOutL > MAX_STEP) {
            leftOut = lastOutL + MAX_STEP;
        }
        else if(leftOut - lastOutL < -MAX_STEP) {
            leftOut = lastOutL - MAX_STEP;
        }

        if(rightOut - lastOutR > MAX_STEP) {
            rightOut = lastOutR + MAX_STEP;
        }
        else if(rightOut - lastOutR < -MAX_STEP) {
            rightOut = lastOutR - MAX_STEP;
        }

        // Set motors to output
        leftBase.SetPercent(leftOut);
        rightBase.SetPercent(-rightOut);

        // Store output for slew rate
        lastOutL = leftOut;
        lastOutR = rightOut;

        // Sleep for set time
        Sleep(LOOP_TIME);

        if(leftTarget - leftEnc.Counts() < 20) {
            leftDone = true;
        }
        if(rightTarget - rightEnc.Counts() < 20) {
            rightDone = true;
        }
        if(leftDone && rightDone) {
            done = true;
        }
    }

    lastOutR = 50;

    rightBase.SetPercent(-50);
    Sleep(250);

    timeDrive(-75, 400);

    // Stop motors
    leftBase.SetPercent(0);
    rightBase.SetPercent(0);
}

int main(void)
{
    float x,y;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    AnalogInputPin cds(FEHIO::P0_7);

    float kP = 0.5;

    int distance = 12;

    int option = 0;

    bool statusDisplayed = false;
    int status = 0;

    while(true)
    {
        if(Accel.X() > 0.3) {
            if(status != SETTING) {
                statusDisplayed = false;
                status = SETTING;
            }
        }
        else if(Accel.Y() < -0.3) {
            if(status != ADJUST) {
                statusDisplayed = false;
                status = ADJUST;
            }
        }
        else if(Accel.Y() > 0.3) {
            if(status != DISTANCE) {
                statusDisplayed = false;
                status = DISTANCE;
            }
        }
        else {
            if(status != RUN) {
                statusDisplayed = false;
                status = RUN;
            }
        }

        if(!statusDisplayed) {
            switch(status) {
            case SETTING:
                LCD.Clear(FEHLCD::Black);
                LCD.WriteLine("Mode selection");
                switch(option) {
                    case AUTO_DRIVE:
                        LCD.WriteRC("autoDrive", 2, 0);
                    break;
                    case AUTO_TURN:
                        LCD.WriteRC("autoTurn ", 2, 0);
                    break;
                    case AUTO_SWEEP:
                        LCD.WriteRC("autoSweep", 2, 0);
                    break;
                    case TOKEN:
                        LCD.WriteRC("Token", 2, 0);
                    break;
                }
            break;
            case RUN:
                LCD.Clear(FEHLCD::Black);
                LCD.WriteLine("Touch to run");
            break;
            case DISTANCE:
                LCD.Clear(FEHLCD::Black);
                LCD.WriteLine("Adjusting distance");
                LCD.WriteRC(distance, 2, 0);
            break;
            case ADJUST:
                LCD.Clear(FEHLCD::Black);
                LCD.WriteLine("Adjusting kP");
                LCD.WriteRC(kP, 2, 0);
            break;
            }
            statusDisplayed = true;
        }

        while(LCD.Touch(&x, &y)) {
            switch(status) {
            case SETTING:
                option++;
                if(option > 3) {
                    option = 0;
                }
                switch(option) {
                    case AUTO_DRIVE:
                        LCD.WriteRC("autoDrive", 2, 0);
                    break;
                    case AUTO_TURN:
                        LCD.WriteRC("autoTurn ", 2, 0);
                    break;
                    case AUTO_SWEEP:
                        LCD.WriteRC("autoSweep", 2, 0);
                    break;
                    case TOKEN:
                        LCD.WriteRC("Token    ", 2, 0);
                    break;
                }
                while(LCD.Touch(&x, &y));
            break;
            case RUN:
                switch(option) {
                    case AUTO_DRIVE:
                        if(distance > 0) {
                            autoDriveF(distance, kP);
                        }
                        else {
                            autoDriveB(-distance, kP);
                        }
                    break;
                    case AUTO_TURN:
                        if(distance > 0) {
                            autoTurnR(distance, kP);
                        }
                        else {
                            autoTurnL(-distance, kP);
                        }
                    break;
                    case AUTO_SWEEP:
                        if(distance > 0) {
                            autoSweepR(distance, kP);
                        }
                        else {
                            autoSweepL(-distance, kP);
                        }
                    break;
                    case TOKEN:
                        moveToToken(kP);
                    break;
                }
            break;
            case ADJUST:
                if(x > 160) {
                    kP += 0.1;
                    LCD.WriteRC(kP, 2, 0);
                    while(LCD.Touch(&x, &y));
                    Sleep(100);
                }
                else {
                    kP -= 0.1;
                    if(kP < 0) {
                        kP = 0;
                    }
                    LCD.WriteRC(kP, 2, 0);
                    while(LCD.Touch(&x, &y));
                    Sleep(100);
                }
            break;
            case DISTANCE:
                if(x > 160) {
                    distance += 1;
                }
                else {
                    distance -= 1;
                }
                LCD.WriteRC("   ", 2, 0);
                LCD.WriteRC(distance, 2, 0);
                while(LCD.Touch(&x, &y));
                Sleep(100);
            break;
            }
        }

        LCD.WriteRC("        ", 4, 0);
        LCD.WriteRC(cds.Value(), 4, 0);

        Sleep(100);
    }
    return 0;
}
