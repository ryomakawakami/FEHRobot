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

#define MAX_SPEED 80

#define MAX_STEP 7  // Max change per iteration
#define LOOP_TIME 0.020   // 20 ms, 50 Hz

#define TICKS_PER_INCH 28 // Conversion from encoder ticks to in

enum {
    RUN,
    ADJUST,
    DISTANCE,
    SETTING,
    SERVO
};

enum {
    AUTO_DRIVE,
    AUTO_TURN,
    AUTO_SWEEP,
    TOKEN,
    UP_RAMP,
    AUTO_DRIVE_SLOW
};

// Declare motors
FEHMotor leftBase(FEHMotor::Motor0, 9);
FEHMotor rightBase(FEHMotor::Motor1, 9);

// Declare encoders
DigitalEncoder rightEnc(FEHIO::P0_0);
DigitalEncoder leftEnc(FEHIO::P1_0);

// Declare servo
FEHServo armServo(FEHServo::Servo0);

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
    PID leftPID(kP, 0.01, 0, 0), rightPID(kP, 0.01, 0, 0), arcPID(1, 0, 0, 0), driftPID(1, 0, 0, 0);

    bool done = false;
    float leftOut, rightOut, driftOut, outL, outR;
    float lastOutL = 0, lastOutR = 0;
    int left, right;

    float rightTarget = 15.5 * TICKS_PER_INCH;
    float leftTarget = 21.5 * TICKS_PER_INCH;
    float errorTarget = 0;

    // Consider allowing for accumulating error
    leftEnc.ResetCounts();
    rightEnc.ResetCounts();

    // Initialize left and right PID
    leftPID.initialize();
    rightPID.initialize();
    arcPID.initialize();
    driftPID.initialize();

    while(!done) {
        // Store encoder values
        left = leftEnc.Counts();
        right = rightEnc.Counts();

        // Set drift target
        errorTarget = left * 160 / 370;
        if (errorTarget > 160) {
            errorTarget = 160;
            driftOut = driftPID.calculate(errorTarget, left - right);
        }
        else {
            driftOut = arcPID.calculate(errorTarget, left - right);
        }

        // Position PID
        leftOut = leftPID.calculate(leftTarget, left);
        rightOut = rightPID.calculate(rightTarget, right);

        outL = leftOut + driftOut;
        outR = rightOut - driftOut;

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

        if(leftTarget - leftEnc.Counts() < 0) {
            done = true;
        }

        LCD.Write(left-right);
    }

    // Stop motors
    leftBase.SetPercent(0);
    rightBase.SetPercent(0);

    LCD.WriteLine(leftTarget - leftEnc.Counts());
    LCD.WriteLine(rightTarget - rightEnc.Counts());
}

void autoDriveFSlow(float target, float kP) {
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

        // Make sure output is between minimum and maximum speed (prevent division by 0 too)
        if(outL != 0) {
            if(fabs(outL) < MIN_SPEED) {
                outL = MIN_SPEED * outL / fabs(outL);
            }
            else if(fabs(outL) > 30) {
                outL = 30 * outL / fabs(outL);
            }
        }
        if(outR != 0) {
            if(fabs(outR) < MIN_SPEED) {
                outR = MIN_SPEED * outR / fabs(outR);
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

void autoDriveBSlow(float target, float kP) {
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
        LCD.WriteLine(target - leftEnc.Counts());
        LCD.WriteLine(target - rightEnc.Counts());
    }

    // Stop motors
    leftBase.SetPercent(0);
    rightBase.SetPercent(0);
}

void autoSweepLB(float target, float kP) {
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

void moveToToken2(float kP) {
    autoDriveBSlow(4.7, kP);
    autoSweepLB(5.5, kP);
    autoDriveB(13, kP);
}

void upRamp() {
    setBase(50);
    while(Accel.Y() < 0.25) {
        Sleep(100);
    }
    while(Accel.Y() > 0.25);
    Sleep(500);
    setBase(0);
}

void downRamp() {
    timeDrive(50, 1000);
    timeDrive(15, 1000);
    Sleep(250);
    timeDrive(50, 500);
    timeDrive(100, 3000);
}

int main(void)
{
    float x,y;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    AnalogInputPin cds(FEHIO::P0_7);

    float kP = 0.4;

    int distance = 12;

    int option = 0;

    int angle = 0;

    bool statusDisplayed = false;
    int status = 0;

    armServo.SetMin(738);
    armServo.SetMax(2500);

    while(true)
    {
        if(Accel.X() > 0.3) {
            if(status != SETTING) {
                statusDisplayed = false;
                status = SETTING;
            }
        }
        else if(Accel.X() < -0.3) {
            if(status != SERVO) {
                statusDisplayed = false;
                status = SERVO;
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
                        LCD.WriteRC("Token    ", 2, 0);
                    break;
                    case UP_RAMP:
                        LCD.WriteRC("Ramp     ", 2, 0);
                    break;
                    case AUTO_DRIVE_SLOW:
                        LCD.WriteRC("Slow     ", 2, 0);
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
            case SERVO:
                LCD.Clear(FEHLCD::Black);
               LCD.WriteLine("Servo");
               LCD.WriteRC(angle, 2, 0);
            break;
            }
            statusDisplayed = true;
        }

        while(LCD.Touch(&x, &y)) {
            switch(status) {
            case SETTING:
                option++;
                if(option > 5) {
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
                    case UP_RAMP:
                        LCD.WriteRC("Ramp     ", 2, 0);
                    break;
                    case AUTO_DRIVE_SLOW:
                        LCD.WriteRC("Slow     ", 2, 0);
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
                        if(distance > 0) {
                            moveToToken(kP);
                        }
                        else {
                            moveToToken2(kP);
                        }
                    break;
                    case UP_RAMP:
                        if(distance > 0) {
                            upRamp();
                        }
                        else {
                            downRamp();
                        }
                    break;
                    case AUTO_DRIVE_SLOW:
                        if(distance > 0) {
                            autoDriveFSlow(distance, kP);
                        }
                        else {
                            autoDriveBSlow(-distance, kP);
                        }
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
            case SERVO:
                if(x > 160) {
                    angle += 10;
                    if(angle > 180) {
                        angle = 180;
                    }
                    LCD.WriteRC("    ", 2, 0);
                    LCD.WriteRC(angle, 2, 0);
                    while(LCD.Touch(&x, &y));
                    Sleep(100);
                }
                else {
                    angle -= 10;
                    if(angle < 0) {
                        angle = 0;
                    }
                    LCD.WriteRC("    ", 2, 0);
                    LCD.WriteRC(angle, 2, 0);
                    while(LCD.Touch(&x, &y));
                    Sleep(100);
                }
                armServo.SetDegree(angle);
            break;
            }
        }

        LCD.WriteRC("        ", 4, 0);
        LCD.WriteRC(cds.Value(), 4, 0);
        LCD.WriteRC("        ", 6, 0);
        LCD.WriteRC(leftEnc.Counts(), 6, 0);
        LCD.WriteRC("        ", 8, 0);
        LCD.WriteRC(rightEnc.Counts(), 8, 0);

        Sleep(100);
    }
    return 0;
}
