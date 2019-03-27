#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHAccel.h>
#include <FEHRPS.h>
#include "pidlib.h"

#define MIN_SPEED 8
#define MIN_SPEED_TURNING 13
#define MIN_SPEED_SWEEP 13

#define MAX_SPEED 60

#define MAX_STEP 7  // Max change per iteration
#define LOOP_TIME 0.020   // 20 ms, 50 Hz

#define KP_DRIVE 0.4
#define KP_TURN 0.4
#define KP_SWEEP 0.6

#define TICKS_PER_INCH 28 // Conversion from encoder ticks to in

#define NO_LIGHT_THRESHOLD 1.9 // 1.5+ is no light
#define BLUE_LIGHT_THRESHOLD 0.8 // 0.8 to 1.5 is blue light

#define PI 3.1415926536

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
// MAX_STEP is slew rate limit (7%)
// LOOP_TIME is time per update (20 ms)
void autoDriveF(float target) {
    PID basePID(KP_DRIVE, 0.01, 0, 0), driftPID(1, 0, 0, 0);

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

        // Make sure output is between minimum and maximum speed (prevent division by 0 too)
        if(outL != 0) {
            if(fabs(outL) < MIN_SPEED) {
                outL = MIN_SPEED * outL / fabs(outL);
            }
            else if(fabs(outL) > MAX_SPEED) {
                outL = MAX_SPEED * outL / fabs(outL);
            }
        }
        if(outR != 0) {
            if(fabs(outR) < MIN_SPEED) {
                outR = MIN_SPEED * outR / fabs(outR);
            }
            else if(fabs(outR) > MAX_SPEED) {
                outR = MAX_SPEED * outR / fabs(outR);
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
    PID basePID(KP_DRIVE, 0.01, 0, 0), driftPID(1, 0, 0, 0);

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

        // Make sure output is between minimum and maximum speed (prevent division by 0 too)
        if(outL != 0) {
            if(fabs(outL) < MIN_SPEED) {
                outL = MIN_SPEED * outL / fabs(outL);
            }
            else if(fabs(outL) > MAX_SPEED) {
                outL = MAX_SPEED * outL / fabs(outL);
            }
        }
        if(outR != 0) {
            if(fabs(outR) < MIN_SPEED) {
                outR = MIN_SPEED * outR / fabs(outR);
            }
            else if(fabs(outR) > MAX_SPEED) {
                outR = MAX_SPEED * outR / fabs(outR);
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
    PID basePID(KP_TURN, 0.01, 0, 0), driftPID(1, 0, 0, 0);

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

        // Make sure output is between minimum and maximum speed (prevent division by 0 too)
        if(outL != 0) {
            if(fabs(outL) < MIN_SPEED_TURNING) {
                outL = MIN_SPEED_TURNING * outL / fabs(outL);
            }
            else if(fabs(outL) > MAX_SPEED) {
                outL = MAX_SPEED * outL / fabs(outL);
            }
        }
        if(outR != 0) {
            if(fabs(outR) < MIN_SPEED_TURNING) {
                outR = MIN_SPEED_TURNING * outR / fabs(outR);
            }
            else if(fabs(outR) > MAX_SPEED) {
                outR = MAX_SPEED * outR / fabs(outR);
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
    PID basePID(KP_TURN, 0.01, 0, 0), driftPID(1, 0, 0, 0);

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

        // Make sure output is between minimum and maximum speed (prevent division by 0 too)
        if(outL != 0) {
            if(fabs(outL) < MIN_SPEED_TURNING) {
                outL = MIN_SPEED_TURNING * outL / fabs(outL);
            }
            else if(fabs(outL) > MAX_SPEED) {
                outL = MAX_SPEED * outL / fabs(outL);
            }
        }
        if(outR != 0) {
            if(fabs(outR) < MIN_SPEED_TURNING) {
                outR = MIN_SPEED_TURNING * outR / fabs(outR);
            }
            else if(fabs(outR) > MAX_SPEED) {
                outR = MAX_SPEED * outR / fabs(outR);
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
            else if(fabs(out) > MAX_SPEED) {
                out = MAX_SPEED * out / fabs(out);
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
            else if(fabs(out) > MAX_SPEED) {
                out = MAX_SPEED * out / fabs(out);
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
            else if(fabs(out) > MAX_SPEED) {
                out = MAX_SPEED * out / fabs(out);
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

void autoDriveFSlow(float target) {
    PID basePID(0.5, 0.01, 0, 0), driftPID(1, 0, 0, 0);

    bool done = false;
    float driveOut, driftOut;
    float outL, outR, lastOutL = 0, lastOutR = 0;
    float avgEnc;
    float startTime = TimeNow();

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

        // Make sure output is between minimum and maximum speed (prevent division by 0 too)
        if(outL != 0) {
            if(fabs(outL) < 20) {
                outL = 20 * outL / fabs(outL);
            }
            else if(fabs(outL) > 40) {
                outL = 40 * outL / fabs(outL);
            }
        }
        if(outR != 0) {
            if(fabs(outR) < 20) {
                outR = 20 * outR / fabs(outR);
            }
            else if(fabs(outR) > 40) {
                outR = 40 * outR / fabs(outR);
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

        if((target - avgEnc < 0) || (TimeNow() - startTime) > 1.5) {
            done = true;
        }
        LCD.WriteLine(target - leftEnc.Counts());
        LCD.WriteLine(target - rightEnc.Counts());
    }

    // Stop motors
    leftBase.SetPercent(0);
    rightBase.SetPercent(0);
}

void autoDriveBSlow(float target) {
    PID basePID(0.5, 0.01, 0, 0), driftPID(1, 0, 0, 0);

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

        // Make sure output is between minimum and maximum speed (prevent division by 0 too)
        if(outL != 0) {
            if(fabs(outL) < MIN_SPEED) {
                outL = MIN_SPEED * outL / fabs(outL);
            }
            else if(fabs(outL) > 40) {
                outL = 40 * outL / fabs(outL);
            }
        }
        if(outR != 0) {
            if(fabs(outR) < MIN_SPEED) {
                outR = MIN_SPEED * outR / fabs(outR);
            }
            else if(fabs(outR) > 40) {
                outR = 40 * outR / fabs(outR);
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
        LCD.WriteLine(target - leftEnc.Counts());
        LCD.WriteLine(target - rightEnc.Counts());
    }

    // Stop motors
    leftBase.SetPercent(0);
    rightBase.SetPercent(0);
}

void setBase(int power) {
    leftBase.SetPercent(-power);
    rightBase.SetPercent(power);
}

void setBaseOff(int powerL, int powerR) {
    leftBase.SetPercent(-powerL);
    rightBase.SetPercent(powerR);
}

void setTurn(int power) {
    leftBase.SetPercent(-power);
    rightBase.SetPercent(-power);
}

void timeDrive(int power, int time) {
    setBase(power);
    Sleep(time);
    setBase(0);
}

void timeDriveOff(int power, int time) {
    setBaseOff(power + 10, power);
    Sleep(time);
    setBase(0);
}

void moveToToken() {
    PID basePID(0.5, 0.01, 0, 0), driftPID(1, 0, 0, 0);

    bool leftDone = false, rightDone = false, done = false;
    float leftOut, rightOut, driveOut, driftOut;
    float outL, outR, lastOutL = 0, lastOutR = 0;
    float avgEnc;

    float rightTarget = 6 * TICKS_PER_INCH;
    float leftTarget = rightTarget + 14 * TICKS_PER_INCH;

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

        // Make sure output is between minimum and maximum speed (prevent division by 0 too)
        if(outL != 0) {
            if(fabs(outL) < MIN_SPEED) {
                outL = MIN_SPEED * outL / fabs(outL);
            }
            else if(fabs(outL) > MAX_SPEED) {
                outL = MAX_SPEED * outL / fabs(outL);
            }
        }
        if(outR != 0) {
            if(fabs(outR) < MIN_SPEED) {
                outR = MIN_SPEED * outR / fabs(outR);
            }
            else if(fabs(outR) > 60) {
                outR = 60 * outR / fabs(outR);
            }
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

    // Stop motors
    leftBase.SetPercent(0);
    rightBase.SetPercent(0);
}

void setAngle(float theta) {
    bool good = false;
    while (!good) {
        float error = RPS.Heading() - theta;
        if (error > 180) {
            error -= 360;
        }
        if (fabs(error) < 0.5) {
            good = true;
        } else {
            if (error > 0) {
                setTurn(15);
            } else {
                setTurn(-15);
            }
            Sleep(25);
            //Sleep(error * 5);
            setTurn(0);
        }
        Sleep(100);
    }
}

void alignRobot() {
    setBase(-20);
    Sleep(500);
    setBase(0);

    while (RPS.Y() < 0) {
        setBase(-20);
        Sleep(100);
        setBase(0);
        Sleep(100);
    }

/*
    float xCoord = RPS.X() - 32.5, yCoord = RPS.Y() - 52, theta;

    if(yCoord == 0) {
        theta = -1 * atan(xCoord / (yCoord + 0.001)) * 180 / PI;
    }
    else {
        theta = -1 * atan(xCoord / yCoord) * 180 / PI;
    }
*/

    //setAngle(theta);

    float currentX = RPS.X();
    if (currentX > 33) {
        setAngle(-5);
    } else if (currentX < 29) {
        setAngle(5);
    } else {
        setAngle(0);
    }

    //setAngle(0);

    //autoDriveBSlow(pow(xCoord, 2) + pow(yCoord, 2));

    while (RPS.Y() > 52) {
        setBase(-20);
        Sleep(100);
        setBase(0);
        Sleep(100);
    }

    setAngle(0);

    /*
    setAngle(0);
    while (RPS.Y() > 54) {
        setBase(-20);
        Sleep(100);
        setBase(0);
        Sleep(100);
    }
    setAngle(0);
    while (RPS.Y() > 52) {
        setBase(-20);
        Sleep(100);
        setBase(0);
        Sleep(100);
    }
    */

    /*
    while (RPS.Y() > 52) {
        setBase(-20);
        Sleep(100);
        setBase(0);
        Sleep(100);
    }

    bool good = false;
    while (!good) {
        float heading = RPS.Heading();
        if (heading > 180) {
            heading -= 360;
        }
        if (fabs(heading) < 0.5) {
            good = true;
        } else {
            if (heading > 0) {
                setTurn(20);
            } else {
                setTurn(-20);
            }
            Sleep(50);
            setTurn(0);
            Sleep(50);
        }
    }
    */
}

void upRamp() {
    setAngle(-4);
    Sleep(250);
    // 30
    setBase(40);
    while(Accel.Y() < 0.25) {
        Sleep(10);
    }
    // 40
    setBase(50);
    while(Accel.Y() > 0.25) {
        Sleep(10);
    }
    // 900
    Sleep(600);
    setBase(0);
    Sleep(250);
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
    armServo.SetDegree(90);

    RPS.InitializeTouchMenu();

    armServo.SetMin(738);
    armServo.SetMax(2500);

    float x, y;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    // Wait for start light or for 30 seconds
    float startTime = TimeNow();
    while((TimeNow() - startTime < 30) && (cds.Value() > NO_LIGHT_THRESHOLD)) {
        Sleep(50);
    }

    // Move to blue button
    autoDriveB(7);
    autoTurnL(8.45);
    autoDriveF(12);

    // Hit blue button
    autoSweepR(11.15);
    timeDrive(-30, 1000);
    autoDriveF(7.4);

    // Move up ramp
    upRamp();
    alignRobot();

    // Line up with foosball
    autoDriveF(6.1);
    autoTurnL(1.55);
    autoDriveF(8.2);
    autoTurnL(4);
    //autoDriveF(1.5);

    armServo.SetDegree(178);
    Sleep(250);

    autoDriveFSlow(9.5);

    armServo.SetDegree(90);
    Sleep(250);


    autoDriveF(3.75);
    autoTurnL(2.8);
    autoDriveF(6.25);
    autoTurnL(2.25);

    //autoDriveF(60);
    timeDrive(50, 1000);
    timeDrive(15, 1000);
    Sleep(250);
    timeDrive(50, 500);
    timeDriveOff(80, 3000);

    /*
    autoTurnR(1.55);
    autoDriveB(9.5);
    autoTurnL(7.1);
    autoDriveF(4);

    setAngle(0);
    while (RPS.Y() > 50) {
        setBase(-20);
        Sleep(100);
        setBase(0);
        Sleep(100);
    }

    autoDriveF(5);
    setAngle(0);
    timeDrive(50, 1000);

    setAngle(0);
    autoTurnR(55);

    timeDrive(75, 2000);
    */

    return 0;
}