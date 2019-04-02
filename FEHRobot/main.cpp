#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHAccel.h>
#include <FEHRPS.h>
#include <FEHBattery.h>
#include "pidlib.h"
//#include "patrick.h"

#define MIN_SPEED 10
#define MIN_SPEED_TURNING 14
#define MIN_SPEED_SWEEP 16

#define MAX_SPEED 80

#define MAX_STEP 7  // Max change per iteration
#define LOOP_TIME 0.020   // 20 ms, 50 Hz

#define KP_DRIVE 0.4
#define KP_TURN 0.4
#define KP_SWEEP 0.6

#define TICKS_PER_INCH 28 // Conversion from encoder ticks to in

#define NO_LIGHT_THRESHOLD 1.6 // 1.5+ is no light
#define BLUE_LIGHT_THRESHOLD 0.95 // 0.8 to 1.5 is blue light

#define ARM_DOWN 178
#define ARM_UP 90

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

// Needed for getting RPS x coordinate after climbing ramp
float xPos = 0;

// PID control loop
// target is desired encoder count
// Position PID when some distance away
// Drift PID and slew rate are constantly active
// Ends function once at location
// MAX_STEP is slew rate limit (7%)
// LOOP_TIME is time per update (20 ms)
void autoDriveF(float target) {
    PID basePID(KP_DRIVE, 0, 0, 0), driftPID(1, 0, 0, 0);

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
    PID basePID(KP_DRIVE, 0, 0, 0), driftPID(1, 0, 0, 0);

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
    PID basePID(KP_TURN, 0, 0, 0), driftPID(1, 0, 0, 0);

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
    PID basePID(KP_TURN, 0, 0, 0), driftPID(1, 0, 0, 0);

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
    PID basePID(KP_SWEEP, 0, 0, 0);

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
    PID basePID(KP_SWEEP, 0, 0, 0);

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
    PID basePID(KP_SWEEP, 0, 0, 0);

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
            else if(fabs(out) > 25) {
                out = 25 * out / fabs(out);
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

void autoDriveBFast(float target) {
    PID basePID(KP_DRIVE, 0, 0, 0), driftPID(1, 0, 0, 0);

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
            else if(fabs(outL) > 90) {
                outL = 90 * outL / fabs(outL);
            }
        }
        if(outR != 0) {
            if(fabs(outR) < MIN_SPEED) {
                outR = MIN_SPEED * outR / fabs(outR);
            }
            else if(fabs(outR) > 90) {
                outR = 90 * outR / fabs(outR);
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

void autoDriveFSlow(float target) {
    PID basePID(0.5, 0, 0, 0), driftPID(1, 0, 0, 0);

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
            if(fabs(outL) < 15) {
                outL = 15 * outL / fabs(outL);
            }
            else if(fabs(outL) > 30) {
                outL = 30 * outL / fabs(outL);
            }
        }
        if(outR != 0) {
            if(fabs(outR) < 15) {
                outR = 15 * outR / fabs(outR);
            }
            else if(fabs(outR) > 30) {
                outR = 30 * outR / fabs(outR);
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
    }

    // Stop motors
    leftBase.SetPercent(0);
    rightBase.SetPercent(0);
}

void autoDriveBSlow(float target) {
    PID basePID(0.5, 0, 0, 0), driftPID(1, 0, 0, 0);

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
            else if(fabs(outL) > 25) {
                outL = 25 * outL / fabs(outL);
            }
        }
        if(outR != 0) {
            if(fabs(outR) < MIN_SPEED) {
                outR = MIN_SPEED * outR / fabs(outR);
            }
            else if(fabs(outR) > 25) {
                outR = 25 * outR / fabs(outR);
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
    autoDriveBSlow(4.3);
    autoSweepLB(5.45);
    autoDriveBFast(12);
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

void setAngle180() {
    bool good = false;
    while (!good) {
        float error = RPS.Heading() - 180;
        if (fabs(error) < 0.5) {
            good = true;
        } else {
            if (error > 0) {
                setTurn(15);
            } else {
                setTurn(-15);
            }
            Sleep(25);
            setTurn(0);
        }
        Sleep(100);
    }
}

void upRamp() {
    setBase(50);
    Sleep(1750);

    /*
    setBase(50);
    Sleep(250);

    setBase(75);
    Sleep(750);

    setBase(50);
    Sleep(250);
    */

    setBase(0);

    setAngle(-2);

    timeDrive(25, 750);
    timeDrive(15, 500);

    Sleep(250);

    while (RPS.Y() < 52) {
        setBase(15);
        Sleep(75);
        setBase(0);
        Sleep(75);
    }

    Sleep(400);

    while (RPS.Y() < 52) {
        setBase(15);
        Sleep(75);
        setBase(0);
        Sleep(75);
    }

    xPos = RPS.X() - 29.9;

    timeDrive(-15, 750);

    setAngle(0);
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
    armServo.SetMin(738);
    armServo.SetMax(2500);
    armServo.SetDegree(ARM_UP);

    RPS.InitializeTouchMenu();

    float x, y;
    float postRampPosition = 0;

    int endL = 0, endR = 0;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    // Starting action
    Sleep(250);
    bool moveOn = false, setupRPS = false;
    while (!moveOn) {
        if (LCD.Touch(&x, &y)) {
            if (x > 160) {
                setupRPS = true;
            }
            else {
                moveOn = true;
            }
        }
        if (setupRPS) {
            LCD.Clear(FEHLCD::Black);
            LCD.WriteLine("RPS Setup");
            bool done = false;
            while(!done) {
                LCD.WriteRC("X:        ", 4, 0);
                LCD.WriteRC(RPS.X(), 0, 2);
                LCD.WriteRC("Y:        ", 6, 0);
                LCD.WriteRC(RPS.Y(), 2, 2);
                if (LCD.Touch(&x, &y)) {
                    done = true;
                    postRampPosition = RPS.X() - 32.2;
                }
                Sleep(100);
            }
            setupRPS = false;
        }
        while (LCD.Touch(&x, &y)) {
            Sleep(10);
        }
        LCD.WriteRC("X:        ", 0, 0);
        LCD.WriteRC(RPS.X(), 0, 2);
        LCD.WriteRC("Y:        ", 2, 0);
        LCD.WriteRC(RPS.Y(), 2, 2);
        LCD.WriteRC("T:        ", 4, 0);
        LCD.WriteRC(RPS.Heading(), 4, 2);
        LCD.WriteRC("L:        ", 8, 0);
        LCD.WriteRC(leftEnc.Counts(), 8, 2);
        LCD.WriteRC("R:        ", 10, 0);
        LCD.WriteRC(rightEnc.Counts(), 10, 2);
        LCD.WriteRC("C:        ", 12, 0);
        LCD.WriteRC(cds.Value(), 12, 2);
        LCD.WriteRC("        ", 12, 12);
        LCD.WriteRC(postRampPosition, 12, 12);
        LCD.WriteRC("       ", 0, 12);
        LCD.WriteRC(Battery.Voltage(), 0 , 12);
        Sleep(50);
    }
    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);
    LCD.WriteRC("Ready :P", 13, 0);

    // Wait for start light or for 30 seconds
    float startTime = TimeNow();
    while((TimeNow() - startTime < 30) && (cds.Value() > NO_LIGHT_THRESHOLD)) {
        Sleep(50);
    }

    // Move to token and score
    moveToToken();

    // Move to DDR light
    autoDriveF(11.75);
    setAngle180();
    autoTurnL(5.15);
    autoDriveF(15);

    // Read DDR light (default blue)
    switch(findColor()) {
        case RED_LIGHT:
            LCD.WriteLine("I READ RED");
            autoDriveB(7.5);
            autoSweepR(11.2);
            timeDrive(-20, 6250);
            autoDriveF(1);
            autoTurnR(3.1);
            autoDriveF(5);
            autoSweepR(5.8);
        break;
        case BLUE_LIGHT:
            LCD.WriteLine("Boo blue");
        default:
            autoDriveB(2);
            autoSweepR(11.2);
            timeDrive(-20, 6250);
            autoDriveF(7.2);
        break;
    }

    // Move to ramp and go up
    setAngle(-2);
    upRamp();

    // Move to foosball, adjusting if necessary
    autoDriveF(6.3);
    autoTurnL(1.85);
    autoDriveF(8.7);
    autoTurnL(3.3);

    // Figure out offset and correct
    float offset = xPos - postRampPosition;
    Sleep(25);
    if (offset > 0) {
        LCD.WriteLine("FORWARD");
        LCD.WriteLine(offset);
        autoDriveF(offset);
        Sleep(25);
    }
    else {
        LCD.WriteLine("BACKWARD");
        LCD.WriteLine(offset);
        autoDriveB(-offset);
        Sleep(25);
    }

    // Score foosball
    armServo.SetDegree(ARM_DOWN);
    Sleep(250);
    autoDriveFSlow(9.75);
    endL = leftEnc.Counts();
    endR = rightEnc.Counts();

    LCD.WriteLine(endL);
    LCD.WriteLine(endR);

    armServo.SetDegree(ARM_UP);
    Sleep(250);

    leftEnc.ResetCounts();
    rightEnc.ResetCounts();
    Sleep(25);

    if (endL > endR) {
        rightBase.SetPercent(MIN_SPEED_SWEEP);
        while (rightEnc.Counts() < endL - endR) {
            Sleep(10);
        }
        rightBase.SetPercent(0);
    }
    else {
        leftBase.SetPercent(MIN_SPEED_SWEEP);
        while (leftEnc.Counts() < endR - endL) {
            Sleep(10);
        }
        leftBase.SetPercent(0);
    }

    // Move to lever
    autoDriveF(2);
    autoSweepR(5.7);
    autoDriveF(5.6);

    // Score lever
    armServo.SetDegree(ARM_DOWN);
    Sleep(250);
    armServo.SetDegree(ARM_UP);

    // Move to ramp
    autoSweepR(5.8);
    autoDriveF(12);
    setAngle180();
    timeDrive(15, 1250);

    // Move down ramp to final button
    timeDrive(50, 500);
    timeDriveOff(80, 3000);
}
