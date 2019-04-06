#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHAccel.h>
#include <FEHRPS.h>
#include <FEHBattery.h>
#include "plib.h"

#define MIN_SPEED 10
#define MIN_SPEED_TURNING 16
#define MIN_SPEED_SWEEP 18

#define MAX_SPEED 60

#define MAX_STEP 7  // Max change per iteration
#define LOOP_TIME 0.020   // 20 ms, 50 Hz

#define KP_DRIVE 0.4
#define KP_TURN 0.4
#define KP_SWEEP 0.6
#define KP_DRIFT 0.5

#define TICKS_PER_INCH 28 // Conversion from encoder ticks to in

#define NO_LIGHT_THRESHOLD 1.7 // 1.7+ is no light
#define BLUE_LIGHT_THRESHOLD 0.95 // 0.95 to 1.7 is blue light

#define ARM_DOWN 156
#define ARM_UP 71

//#define ARM_DOWN 169
//#define ARM_UP 84

#define rpsSetupX 31.9
#define rpsSetupY 52

#define rpsTargetX 29.8
#define rpsTargetY 52

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

// Needed for getting RPS coordinates after climbing ramp
float xPos = 0, yPos = 0;

// Stores ideal 0 degree position for course
float zeroDegrees = 0;

// Arm positions
int armUp = ARM_UP, armDown = ARM_DOWN;

// Simple PID control loop
// target is desired encoder count
// Position PID when some distance away
// Drift PID and slew rate are constantly active
// Ends function once at location
// MAX_STEP is slew rate limit (7%)
// LOOP_TIME is time per update (20 ms)
void autoDriveF(float target) {
    PID basePID(KP_DRIVE), driftPID(KP_DRIFT);

    bool done = false;
    float driveOut, driftOut;
    float outL, outR, lastOutL = 0, lastOutR = 0;
    float avgEnc;

    target *= TICKS_PER_INCH;

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
    PID basePID(KP_DRIVE), driftPID(KP_DRIFT);

    bool done = false;
    float driveOut, driftOut;
    float outL, outR, lastOutL = 0, lastOutR = 0;
    float avgEnc;

    target *= TICKS_PER_INCH;

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
    PID basePID(KP_TURN), driftPID(KP_DRIFT);

    bool done = false;
    float driveOut, driftOut;
    float outL, outR, lastOutL = 0, lastOutR = 0;
    float avgEnc;

    target *= TICKS_PER_INCH;

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
    PID basePID(KP_TURN), driftPID(KP_DRIFT);

    bool done = false;
    float driveOut, driftOut;
    float outL, outR, lastOutL = 0, lastOutR = 0;
    float avgEnc;

    target *= TICKS_PER_INCH;

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
    PID basePID(KP_SWEEP);

    bool done = false;
    float out, lastOut = 0;
    float counts;

    target *= TICKS_PER_INCH;

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
    PID basePID(KP_SWEEP);

    bool done = false;
    float out, lastOut = 0;
    float counts;

    target *= TICKS_PER_INCH;

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
    PID basePID(KP_SWEEP);

    bool done = false;
    float out, lastOut = 0;
    float counts;

    target *= TICKS_PER_INCH;

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
    PID basePID(KP_DRIVE), driftPID(KP_DRIFT);

    bool done = false;
    float driveOut, driftOut;
    float outL, outR, lastOutL = 0, lastOutR = 0;
    float avgEnc;

    target *= TICKS_PER_INCH;

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
    PID basePID(KP_DRIVE), driftPID(KP_DRIFT);

    bool done = false;
    float driveOut, driftOut;
    float outL, outR, lastOutL = 0, lastOutR = 0;
    float avgEnc;
    float startTime = TimeNow();

    target *= TICKS_PER_INCH;

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
    PID basePID(KP_DRIVE), driftPID(KP_DRIFT);

    bool done = false;
    float driveOut, driftOut;
    float outL, outR, lastOutL = 0, lastOutR = 0;
    float avgEnc;

    target *= TICKS_PER_INCH;

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

// CCW is positive
void timeTurn(int power, int time) {
    setBaseOff(-power, power);
    Sleep(time);
    setBase(0);
}

void moveToToken() {
    autoDriveBSlow(4.3);
    autoSweepLB(5.4);
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
                setTurn(20);
            } else {
                setTurn(-20);
            }
            Sleep(25);
            setTurn(0);
        }
        Sleep(100);
    }
}

void setAngle180(float theta) {
    bool good = false;
    float target = theta - zeroDegrees;
    while (!good) {
        float error = RPS.Heading() - target;
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
    leftBase.SetPercent(-50);
    rightBase.SetPercent(50);
    Sleep(500);

    int leftPower = -70, rightPower = 90;
    float headingAdj = 0;
    long startTime = TimeNowMSec(), onFlat = TimeNowMSec();
    while (TimeNowMSec() - startTime < 2000) {
        headingAdj = RPS.Heading() * 4;
        leftPower = -70 - headingAdj;
        rightPower = 90 - headingAdj;

        if (Accel.Y() > 0.2) {
            onFlat = TimeNowMSec();
        }

        Sleep(50);
    }

    setBase(25);
    Sleep(500);

    setBase(-25);

    while (RPS.Y() < 0) {
        Sleep(50);
    }

    while (RPS.Y() < 0) {
        Sleep(50);
    }

    setBase(0);

    setAngle(zeroDegrees);

    setBase(-15);

    xPos = RPS.X() - 29.8;

    while (xPos < -30) {
        setBase(-15);
        Sleep(50);
        setBase(0);
        Sleep(50);
        xPos = RPS.X() - 29.8;
    }

    Sleep(500);
    setBase(0);

    yPos = RPS.Y() - 52;

    while (yPos < -52) {
        setBase(-15);
        Sleep(50);
        setBase(0);
        Sleep(50);
        yPos = RPS.Y() - 52;
    }
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

void scoreFoosball() {
    int endL = 0, endR = 0;

    // Score foosball
    armServo.SetDegree(armDown);
    Sleep(250);
    autoDriveFSlow(9.75);
    endL = leftEnc.Counts();
    endR = rightEnc.Counts();

    LCD.WriteLine(endL);
    LCD.WriteLine(endR);

    armServo.SetDegree(armUp);
    Sleep(250);

    //alignWheels(273);

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
}

int main(void) {
    armServo.SetMin(738);
    armServo.SetMax(2500);
    armServo.SetDegree(armUp);

    RPS.InitializeTouchMenu();

    float x, y;
    float postRampX = 0, postRampY = 0;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    // Starting action
    Sleep(250);
    bool moveOn = false, setupRPS = false;
    while (!moveOn) {
        if (LCD.Touch(&x, &y)) {
            if (Accel.Y() > 0.25 || Accel.Y() < -0.25) {
                while (Accel.Y() > 0.25) {
                    LCD.Clear(FEHLCD::Black);
                    LCD.WriteRC("        ", 2, 0);
                    LCD.WriteRC(armUp, 2, 0);
                    LCD.WriteRC("        ", 4, 0);
                    LCD.WriteRC(armDown, 4, 0);
                    while (!LCD.Touch(&x, &y)) {
                        Sleep(10);
                    }
                    if (x < 160) {
                        armUp--;
                        armDown--;
                    }
                    else {
                        armUp++;
                        armDown++;
                    }
                    armServo.SetDegree(armUp);
                    while (LCD.Touch(&x, &y)) {
                        Sleep(10);
                    }
                }
            }
            else if (x > 160) {
                setupRPS = true;
                //scoreFoosball();
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
                LCD.WriteRC("X:        ", 2, 0);
                LCD.WriteRC(RPS.X(), 2, 2);
                LCD.WriteRC("Y:        ", 4, 0);
                LCD.WriteRC(RPS.Y(), 4, 2);
                LCD.WriteRC("Y:        ", 6, 0);
                LCD.WriteRC(RPS.Y(), 6, 2);
                if (LCD.Touch(&x, &y)) {
                    done = true;
                    postRampX = RPS.X() - 31.9;
                    postRampY = RPS.Y() - 52;
                    zeroDegrees = RPS.Heading() - 180;
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
        LCD.WriteRC("        ", 11, 12);
        LCD.WriteRC(postRampX, 11, 12);
        LCD.WriteRC("        ", 12, 12);
        LCD.WriteRC(postRampY, 12, 12);
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
    setAngle180(176);
    autoTurnL(5.2);
    autoDriveF(15);

    // Read DDR light (default blue)
    switch(findColor()) {
        case RED_LIGHT:
            LCD.WriteLine("I READ RED");
            autoDriveB(6.5);
            autoSweepR(11.1);
            timeDrive(-20, 5750);
            autoDriveF(1);
            autoTurnR(3);
            autoDriveF(5);
            autoSweepR(6.1);
        break;
        case BLUE_LIGHT:
            LCD.WriteLine("Boo blue");
        default:
            autoDriveB(1.5);
            autoSweepR(11.1);
            timeDrive(-20, 5750);
            autoDriveF(7.2);
        break;
    }

    // Move to ramp and go up
    setAngle(zeroDegrees);
    upRamp();

    // Move to foosball, adjusting if necessary
    float offsetY = yPos - postRampY;
    Sleep(50);

    //autoDriveFSlow(5.48 - offsetY);

    bool leftDone = false, rightDone = false;
    float target = (5.45 - offsetY) * TICKS_PER_INCH;
    leftEnc.ResetCounts();
    rightEnc.ResetCounts();
    Sleep(25);
    leftBase.SetPercent(-20);
    rightBase.SetPercent(20);
    while (!(leftDone || rightDone)) {
        if (leftEnc.Counts() > target) {
            leftBase.SetPercent(0);
            leftDone = true;
        //}
        //if (rightEnc.Counts() > target) {
            rightBase.SetPercent(0);
            rightDone = true;
        }
        Sleep(10);
    }
    Sleep(250);

    //alignWheels(target);

    int leftCounts, rightCounts;
    leftCounts = leftEnc.Counts();
    rightCounts = rightEnc.Counts();
    Sleep(50);

    if (leftCounts > rightCounts) {
        rightBase.SetPercent(MIN_SPEED_SWEEP);
        while (rightEnc.Counts() < leftCounts - rightCounts) {
            Sleep(10);
        }
        rightBase.SetPercent(0);
    }
    else {
        leftBase.SetPercent(MIN_SPEED_SWEEP);
        while (leftEnc.Counts() < rightCounts - leftCounts) {
            Sleep(10);
        }
        leftBase.SetPercent(0);
    }

    autoTurnL(1.8);
    Sleep(50);

    //autoDriveF(8.9);

    leftDone = false, rightDone = false;
    target = 8.45 * TICKS_PER_INCH;
    leftEnc.ResetCounts();
    rightEnc.ResetCounts();
    Sleep(25);
    leftBase.SetPercent(-20);
    rightBase.SetPercent(20);
    while (!(leftDone || rightDone)) {
        if (leftEnc.Counts() > target) {
            leftBase.SetPercent(0);
            leftDone = true;
        //}
        //if (rightEnc.Counts() > target) {
            rightBase.SetPercent(0);
            rightDone = true;
        }
        Sleep(10);
    }
    Sleep(250);

    autoTurnL(2.98);

    // Figure out x offset and correct
    float offsetX = xPos - postRampX;
    Sleep(25);
    if (offsetX > 0) {
        LCD.WriteLine("FORWARD");
        LCD.WriteLine(offsetX);
        autoDriveF(offsetX);
        Sleep(25);
    }
    else {
        LCD.WriteLine("BACKWARD");
        LCD.WriteLine(offsetX);
        autoDriveB(-offsetX);
        Sleep(25);
    }

    // Score foosball
    scoreFoosball();

    // Move to lever
    autoDriveF(2.5);
    autoSweepR(6.5);
    autoDriveF(2);
    //autoDriveF(5.2);

    // Score lever
    armServo.SetDegree(armDown);
    Sleep(250);
    armServo.SetDegree(armUp);

    autoDriveF(3);

    // Move to ramp
    autoSweepR(4);
    autoDriveF(12);
    setAngle180(180);
    timeDrive(15, 1250);

    // Move down ramp to final button
    timeDrive(50, 500);
    timeDrive(80, 2000);

    // Repeatedly back up and ram something
    while (1) {
        timeDrive(-50, 500);
        timeTurn(-20, 250);
        timeDrive(50, 1000);
    }
}
