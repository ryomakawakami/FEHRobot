#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHAccel.h>
#include "pidlib.h"

#define MIN_SPEED 8
#define MIN_SPEED_TURNING 12
#define MIN_SPEED_SWEEP 12

#define MAX_STEP 10  // Max change per iteration
#define LOOP_TIME 0.020   // 20 ms, 50 Hz

#define KP_DRIVE 0.4
#define KP_TURN 0.4
#define KP_SWEEP 0.6

#define TICKS_PER_INCH 28 // Conversion from encoder ticks to in

#define NO_LIGHT_THRESHOLD 1.9 // 1.5+ is no light
#define BLUE_LIGHT_THRESHOLD 0.8 // 0.8 to 1.5 is blue light

enum {
    NO_LIGHT,
    BLUE_LIGHT,
    RED_LIGHT
};

// Declare servo
FEHServo armServo(FEHServo::Servo0);

// Declare motors
FEHMotor leftBase(FEHMotor::Motor0, 9);
FEHMotor rightBase(FEHMotor::Motor1, 9);

// Declare encoders
DigitalEncoder rightEnc(FEHIO::P0_0);
DigitalEncoder leftEnc(FEHIO::P1_0);

// Declare CdS cell
AnalogInputPin cds(FEHIO::P0_7);

// PID control loop
// target is desired encoder count
// Position PID when some distance away
// Drift PID and slew rate are constantly active
// Ends function once at location
// MAX_STEP is slew rate limit (10%)
// LOOP_TIME is time per update (20 ms)
void autoDriveF(float target) {
    PID basePID(KP_DRIVE, 0.01, 0, 0), driftPID(2, 0, 0, 0);

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

        // Make sure output is at least minimum speed (prevent division by 0 too)
        if(outL != 0) {
            if(fabs(outL) < MIN_SPEED) {
                outL = MIN_SPEED * outL / fabs(outL);
            }
        }
        if(outR != 0) {
            if(fabs(outR) < MIN_SPEED) {
                outR = MIN_SPEED * outR / fabs(outR);
            }
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

void autoDriveB(float target) {
    PID basePID(KP_DRIVE, 0.01, 0, 0), driftPID(2, 0, 0, 0);

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

        // Make sure output is at least minimum speed (prevent division by 0 too)
        if(outL != 0) {
            if(fabs(outL) < MIN_SPEED) {
                outL = MIN_SPEED * outL / fabs(outL);
            }
        }
        if(outR != 0) {
            if(fabs(outR) < MIN_SPEED) {
                outR = MIN_SPEED * outR / fabs(outR);
            }
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

void autoTurnL(float target) {
    PID basePID(KP_TURN, 0.01, 0, 0), driftPID(2, 0, 0, 0);

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

        // Make sure output is at least minimum speed (prevent division by 0 too)
        if(outL != 0) {
            if(fabs(outL) < MIN_SPEED_TURNING) {
                outL = MIN_SPEED_TURNING * outL / fabs(outL);
            }
        }
        if(outR != 0) {
            if(fabs(outR) < MIN_SPEED_TURNING) {
                outR = MIN_SPEED_TURNING * outR / fabs(outR);
            }
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

void autoTurnR(float target) {
    PID basePID(KP_TURN, 0.01, 0, 0), driftPID(2, 0, 0, 0);

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

        // Make sure output is at least minimum speed (prevent division by 0 too)
        if(outL != 0) {
            if(fabs(outL) < MIN_SPEED_TURNING) {
                outL = MIN_SPEED_TURNING * outL / fabs(outL);
            }
        }
        if(outR != 0) {
            if(fabs(outR) < MIN_SPEED_TURNING) {
                outR = MIN_SPEED_TURNING * outR / fabs(outR);
            }
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

void autoSweepR(float target) {
    PID basePID(KP_SWEEP, 0.01, 0, 0);

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

        // Make sure output is at least minimum speed (prevent division by 0 too)
        if(out != 0) {
            if(fabs(out) < MIN_SPEED_SWEEP) {
                out = MIN_SPEED_SWEEP * out / fabs(out);
            }
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

void autoSweepL(float target) {
    PID basePID(KP_SWEEP, 0.01, 0, 0);

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

        // Make sure output is at least minimum speed (prevent division by 0 too)
        if(out != 0) {
            if(fabs(out) < MIN_SPEED_SWEEP) {
                out = MIN_SPEED_SWEEP * out / fabs(out);
            }
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

void autoSweepLB(float target) {
    PID basePID(KP_SWEEP, 0.01, 0, 0);

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

        // Make sure output is at least minimum speed (prevent division by 0 too)
        if(out != 0) {
            if(fabs(out) < MIN_SPEED_SWEEP) {
                out = MIN_SPEED_SWEEP * out / fabs(out);
            }
        }

        // Set motors to output
        leftBase.SetPercent(out);

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

void moveToToken() {
    PID basePID(0.5, 0.01, 0, 0), driftPID(2, 0, 0, 0);

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

void upRamp() {
    setBase(30);
    while(Accel.Y() < 0.25) {
        Sleep(100);
    }
    setBase(50);
    while(Accel.Y() > 0.25);
    Sleep(250);
    setBase(0);
}

int findColor() {
    if(cds.Value() > NO_LIGHT_THRESHOLD) {
        return NO_LIGHT;
    }
    else if(cds.Value() > BLUE_LIGHT_THRESHOLD) {
        return BLUE_LIGHT;
    }
    else {
        return RED_LIGHT;
    }
}

int main(void) {
    armServo.SetMin(748);
    armServo.SetMax(2500);

    float x, y;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    // Wait for start light or for 30 seconds
    float startTime = TimeNow();
    while((TimeNow() - startTime < 30) && (cds.Value() > NO_LIGHT_THRESHOLD)) {
        Sleep(50);
    }

    // Move to token and run into it
    moveToToken();

    return 0;
}
