#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHAccel.h>
#include <FEHRPS.h>
#include <FEHBattery.h>
#include "plib.h"

// Minimum speeds
#define MIN_SPEED 10
#define MIN_SPEED_TURNING 16
#define MIN_SPEED_SWEEP 18

// Maximum movement speed
#define MAX_SPEED 60

// P loop speed
#define LOOP_TIME 0.020

// Slew rate limit per iteration
#define MAX_STEP 7

// kP for movements
#define KP_DRIVE 0.4
#define KP_TURN 0.4
#define KP_SWEEP 0.6
#define KP_DRIFT 0.5

// Conversion from ticks to inches
#define TICKS_PER_INCH 2

// Servo values
#define ARM_DOWN 156
#define ARM_UP 71

// RPS setup position coordinate
#define RPS_SETUP_X 31.9
#define RPS_SETUP_Y 52

// RPS target position coordinate
#define RPS_TARGET_X 29.8
#define RPS_TARGET_Y 52

// RPS angle tolerance
#define EPSILON 0.5

// CdS cell thresholds: Red [0, 0.95], Blue [0.95, 1.7], No Light
#define NO_LIGHT_THRESHOLD 1.7
#define BLUE_LIGHT_THRESHOLD 0.95

// CdS cell light values
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

// Stores 0 degree position for course
float zeroDegrees = 0;

// Arm positions
int armUp = ARM_UP, armDown = ARM_DOWN;

// Servo angle adjustment
void adjustServo() {
    float x, y;

    LCD.Clear(FEHLCD::Black);
    LCD.WriteRC("        ", 2, 0);
    LCD.WriteRC(armUp, 2, 0);
    LCD.WriteRC("        ", 4, 0);
    LCD.WriteRC(armDown, 4, 0);

    // Wait for touch
    while (!LCD.Touch(&x, &y)) {
        Sleep(10);
    }

    // Lower by 1 degree if left side of screen
    if (x < 160) {
        armUp--;
        armDown--;
    }
    // Raise by 1 degree if right side
    else {
        armUp++;
        armDown++;
    }

    armServo.SetDegree(armUp);

    while (LCD.Touch(&x, &y)) {
        Sleep(10);
    }
}

// Displays RPS coordinates
void displayRPS() {
    LCD.WriteRC("X:        ", 0, 0);
    LCD.WriteRC(RPS.X(), 0, 2);
    LCD.WriteRC("Y:        ", 2, 0);
    LCD.WriteRC(RPS.Y(), 2, 2);
    LCD.WriteRC("T:        ", 4, 0);
    LCD.WriteRC(RPS.Heading(), 4, 2);
}

// Displays encoder values, CdS cell value, RPS offset, and voltage
void displayOther(float postRampX, float postRampY) {
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
}

// Simple PID control loop
// target is desired encoder count
// Position PID when some distance away
// DriftPI PID and slew rate are constantly active
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

// Backward
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

// Left turn
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

// Right turn
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

// Left sweep turn
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

// Right sweep turn
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

// Left sweep turn backwards (for token)
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

// Slow forward (for foosball)
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

// Slow backward (for token)
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

// Fast backward (for token)
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

// Sets base at specified power
void setBase(int power) {
    leftBase.SetPercent(-power);
    rightBase.SetPercent(power);
}

// Sets each side of base at specified powers
void setBaseOff(int powerL, int powerR) {
    leftBase.SetPercent(-powerL);
    rightBase.SetPercent(powerR);
}

// Sets base to turn at specified power
void setTurn(int power) {
    leftBase.SetPercent(-power);
    rightBase.SetPercent(-power);
}

// Forward/backward, specified power and time
void timeDrive(int power, int time) {
    setBase(power);
    Sleep(time);
    setBase(0);
}

// Turn, specified power and time
void timeTurn(int power, int time) {
    setBaseOff(-power, power);
    Sleep(time);
    setBase(0);
}

// Slow forward with no P
void slowForward(float target) {
    // Convert target to ticks
    target *= TICKS_PER_INCH;

    // Reset encoder counts
    leftEnc.ResetCounts();
    rightEnc.ResetCounts();
    Sleep(25);

    // Set base to low forward speed
    setBase(20);

    // Continue until left encoder goes over target value
    while (leftEnc.Counts() < target) {
        Sleep(10);
    }

    // Stop base
    setBase(0);

    // Sleep for .25 seconds
    Sleep(250);
}

// RPS heading correction (270 to 90 degrees)
void setAngle(float theta) {
    bool done = false;

    // Offset based on course's zero degrees
    float target = theta - zeroDegrees;

    while (!done) {
        // Find error
        float error = RPS.Heading() - target;

        // Zero crossing correction [270, 90] -> [-90, 90]
        if (error > 180) {
            error -= 360;
        }

        // Check if error is within epsilon
        if (fabs(error) < EPSILON) {
            done = true;
        }
        else {
            // Check direction and make correction based on that
            if (error > 0) {
                setTurn(20);
            }
            else {
                setTurn(-20);
            }

            Sleep(25);

            setTurn(0);
        }

        Sleep(100);
    }
}

// RPS heading correction (90 to 270 degrees)
void setAngle180(float theta) {
    bool done = false;

    // Offset based on course's zero degrees
    float target = theta - zeroDegrees;
    while (!done) {
        // Find error
        float error = RPS.Heading() - target;

        // Check if error is within epsilon
        if (fabs(error) < EPSILON) {
            done = true;
        }
        else {
            // Check direction and make correction based on that
            if (error > 0) {
                setTurn(15);
            }
            else {
                setTurn(-15);
            }

            Sleep(25);

            setTurn(0);
        }

        Sleep(100);
    }
}

// Return color of light
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

// Token movement
void moveToToken() {
    autoDriveBSlow(4.3);
    autoSweepLB(5.4);
    autoDriveBFast(12);
}

// Move to DDR light
void moveToDDR() {
    autoDriveF(11.75);
    setAngle180(176);
    autoTurnL(5.2);
    autoDriveF(15);
}

// Read and score DDR button
void scoreDDR() {
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
}

// Go up ramp
void upRamp() {
    // Correct heading
    setAngle(0);

    // Go slow for 0.5 seconds
    leftBase.SetPercent(-50);
    rightBase.SetPercent(50);
    Sleep(500);

    // Climb ramp at high power while adjusting based on heading (simple P)
    int leftPower = -70, rightPower = 90;
    float headingAdj = 0;
    long startTime = TimeNowMSec();
    // Continue for 2 seconds
    while (TimeNowMSec() - startTime < 2000) {
        headingAdj = RPS.Heading() * 4;

        leftPower -= headingAdj;
        rightPower -= headingAdj;

        Sleep(50);
    }

    // Continue forward for 0.5 seconds to prevent tip
    setBase(25);
    Sleep(500);

    // Back up until in RPS range
    setBase(-25);

    while (RPS.Y() < 0) {
        Sleep(50);
    }

    while (RPS.Y() < 0) {
        Sleep(50);
    }

    setBase(0);

    // Correct angle
    setAngle(0);

    // Back up into steps to align
    setBase(-15);

    // Find x offset while doing so
    xPos = RPS.X() - RPS_TARGET_X;

    // Recheck in case of RPS error
    while (xPos < -RPS_TARGET_X) {
        setBase(-15);
        Sleep(50);
        setBase(0);
        Sleep(50);
        xPos = RPS.X() - RPS_TARGET_X;
    }

    Sleep(500);
    setBase(0);

    // Find ending y position
    yPos = RPS.Y() - RPS_TARGET_Y;
    LCD.WriteLine(yPos);

    // Recheck in case of RPS error
    while (yPos < -RPS_TARGET_Y) {
        setBase(-15);
        Sleep(50);
        setBase(0);
        Sleep(50);
        yPos = RPS.Y() - RPS_TARGET_Y;
    }
}

// Move to foosball
void moveToFoosball(float offsetY) {
    slowForward(5.5 - offsetY);
    autoTurnL(1.8);
    slowForward(8.45);
    autoTurnL(2.98);
}

// Correct x offset
void correctOffsetX(float offsetX) {
    if (offsetX > 0) {
        autoDriveF(offsetX);
        Sleep(25);
    }
    else {
        autoDriveB(-offsetX);
        Sleep(25);
    }
}

// Score foosball
void scoreFoosball() {
    int endL = 0, endR = 0;

    // Move foosball
    armServo.SetDegree(armDown);
    Sleep(250);
    autoDriveFSlow(9.85);

    // Store encoder counts
    endL = leftEnc.Counts();
    endR = rightEnc.Counts();

    // Raise arm
    armServo.SetDegree(armUp);
    Sleep(250);

    // Reset encoders
    leftEnc.ResetCounts();
    rightEnc.ResetCounts();
    Sleep(25);

    // Correct encoder offset
    if (endL > endR) {
        leftBase.SetPercent(MIN_SPEED_SWEEP);
        while (leftEnc.Counts() < endL - endR) {
            Sleep(10);
        }
        leftBase.SetPercent(0);
        endL -= leftEnc.Counts();
    }
    else {
        rightBase.SetPercent(-MIN_SPEED_SWEEP);
        while (rightEnc.Counts() < endR - endL) {
            Sleep(10);
        }
        rightBase.SetPercent(0);
        endR -= rightEnc.Counts();
    }

    // Reset encoders again
    leftEnc.ResetCounts();
    rightEnc.ResetCounts();
}

// Move to lever
void moveToLever() {
    autoDriveF(2.5);
    autoSweepR(6.5);
    autoDriveF(1.9);
}

// Score lever
void scoreLever() {
    armServo.SetDegree(armDown);
    Sleep(250);
    armServo.SetDegree(armUp);
}

// Move to ramp with bump
void moveToRamp() {
    autoDriveF(3);
    autoSweepR(4);
    autoDriveF(12);
    setAngle180(180);
    timeDrive(15, 1250);
}

// Move down ramp and hit final button
void downRamp() {
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

int main(void) {
    // Servo positions
    armServo.SetMin(738);
    armServo.SetMax(2500);
    armServo.SetDegree(armUp);

    // Initialize RPS
    RPS.InitializeTouchMenu();

    // Touchscreen coordinates
    float x, y;

    // Desired post ramp position
    float postRampX = 0, postRampY = 0;

    // Clear display
    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    // Starting action
    Sleep(250);
    bool moveOn = false, setupRPS = false;
    while (!moveOn) {
        // Determine touch position
        if (LCD.Touch(&x, &y)) {

            // Servo calibration (robot tilted)
            if (Accel.Y() > 0.25 || Accel.Y() < -0.25) {
                // Until untilted
                while (Accel.Y() > 0.25) {
                    adjustServo();
                }
            }

            // RPS calibration if right side of screen
            else if (x > 160) {
                setupRPS = true;
            }

            // Final action if left side of screen
            else {
                moveOn = true;
            }
        }

        // RPS calibration
        if (setupRPS) {

            LCD.Clear(FEHLCD::Black);
            LCD.WriteLine("RPS Setup");

            bool done = false;
            while(!done) {
                // Display RPS coordinates
                displayRPS();

                // If touched, store position and end calibration
                if (LCD.Touch(&x, &y)) {
                    done = true;
                    postRampX = RPS.X() - RPS_SETUP_X;
                    postRampY = RPS.Y() - RPS_SETUP_Y;
                    zeroDegrees = RPS.Heading() - 180;
                }

                Sleep(100);
            }
            setupRPS = false;
        }

        // Wait for release
        while (LCD.Touch(&x, &y)) {
            Sleep(10);
        }

        // Update screen
        displayRPS();
        displayOther(postRampX, postRampY);
        Sleep(50);
    }

    // Clear screen
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
    moveToDDR();

    // Read DDR light (default blue)
    scoreDDR();

    // Move to ramp and go up
    upRamp();

    // Move to foosball
    moveToFoosball(yPos - postRampY);

    // Figure out x offset and correct
    correctOffsetX(xPos - postRampX);

    // Score foosball
    scoreFoosball();

    // Move to lever
    moveToLever();

    // Score lever
    scoreLever();

    // Move to ramp with bump
    moveToRamp();

    // Move down ramp
    downRamp();
}
