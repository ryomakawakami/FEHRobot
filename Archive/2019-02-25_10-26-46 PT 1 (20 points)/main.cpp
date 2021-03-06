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

#define MAX_VELOCITY 28 // Max (reasonable) velocity in in/s
#define VELOCITY_RANGE 12 // Range of velocity PID in in

#define TICKS_PER_INCH 28 // Conversion from encoder ticks to in

#define RED_THRESHOLD 0.5 // Threshold for red light vs no light

// Declare servo
FEHServo armServo(FEHServo::Servo0);

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
void autoDriveF(float target) {
    PID basePID(0.5, 0.01, 0, 0), driftPID(2, 0, 0, 0);

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

        // Set motors to output
        leftBase.SetPercent(outL + MIN_SPEED);
        rightBase.SetPercent(-outR - MIN_SPEED);

        // Store output for slew rate
        lastOutL = outL;
        lastOutR = outR;

        // Sleep for set time
        Sleep(LOOP_TIME);

        if(target - avgEnc < 0) {
            done = true;
        }

        LCD.Write(avgEnc);
        LCD.Write("\t");
        LCD.Write(leftEnc.Counts());
        LCD.Write("\t");
        LCD.WriteLine(rightEnc.Counts());

        LCD.Write(outL);
        LCD.Write(" ");
        LCD.WriteLine(outR);
    }

    // Stop motors
    leftBase.SetPercent(-30);
    rightBase.SetPercent(0);

    Sleep(100);

    leftBase.SetPercent(0);
    rightBase.SetPercent(0);
}

// PID control loop
// target is desired encoder count
// Position PID when some distance away
// Drift PID and slew rate are constantly active
// Ends function once at location
// MAX_STEP is slew rate limit (10%)
// LOOP_TIME is time per update (20 ms)
void autoDriveB(float target) {
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
        avgEnc = (leftEnc.Counts() + rightEnc.Counts()) / 2;

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

        // Set motors to output
        leftBase.SetPercent(-outL - MIN_SPEED);
        rightBase.SetPercent(outR + MIN_SPEED);

        // Store output for slew rate
        lastOutL = outL;
        lastOutR = outR;

        // Sleep for set time
        Sleep(LOOP_TIME);

        if(target - avgEnc < 0) {
            done = true;
        }

        LCD.Write(avgEnc);
        LCD.Write("\t");
        LCD.Write(leftEnc.Counts());
        LCD.Write("\t");
        LCD.WriteLine(rightEnc.Counts());

        LCD.Write(outL);
        LCD.Write(" ");
        LCD.WriteLine(outR);
    }

    // Stop motors
    leftBase.SetPercent(10);
    rightBase.SetPercent(-10);

    Sleep(100);

    leftBase.SetPercent(0);
    rightBase.SetPercent(0);
}

// PID control loop
// target is desired encoder count
// Position PID when some distance away
// Drift PID and slew rate are constantly active
// Ends function once at location
// MAX_STEP is slew rate limit (10%)
// LOOP_TIME is time per update (20 ms)
void autoTurnL(float target) {
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
        avgEnc = (leftEnc.Counts() + rightEnc.Counts()) / 2;

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

        // Set motors to output
        leftBase.SetPercent(outL + MIN_SPEED);
        rightBase.SetPercent(outR + MIN_SPEED);

        // Store output for slew rate
        lastOutL = outL;
        lastOutR = outR;

        // Sleep for set time
        Sleep(LOOP_TIME);

        if(target - avgEnc < 0) {
            done = true;
        }

        LCD.Write(avgEnc);
        LCD.Write("\t");
        LCD.Write(leftEnc.Counts());
        LCD.Write("\t");
        LCD.WriteLine(rightEnc.Counts());

        LCD.Write(outL);
        LCD.Write(" ");
        LCD.WriteLine(outR);
    }

    // Stop motors
    leftBase.SetPercent(-10);
    rightBase.SetPercent(-10);

    Sleep(100);

    leftBase.SetPercent(0);
    rightBase.SetPercent(0);
}

// PID control loop
// target is desired encoder count
// Position PID when some distance away
// Drift PID and slew rate are constantly active
// Ends function once at location
// MAX_STEP is slew rate limit (10%)
// LOOP_TIME is time per update (20 ms)
void autoTurnR(float target) {
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
        avgEnc = (leftEnc.Counts() + rightEnc.Counts()) / 2;

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

        // Set motors to output
        leftBase.SetPercent(-outL - MIN_SPEED);
        rightBase.SetPercent(-outR - MIN_SPEED);

        // Store output for slew rate
        lastOutL = outL;
        lastOutR = outR;

        // Sleep for set time
        Sleep(LOOP_TIME);

        if(target - avgEnc < 0) {
            done = true;
        }

        LCD.Write(avgEnc);
        LCD.Write("\t");
        LCD.Write(leftEnc.Counts());
        LCD.Write("\t");
        LCD.WriteLine(rightEnc.Counts());

        LCD.Write(outL);
        LCD.Write(" ");
        LCD.WriteLine(outR);
    }

    // Stop motors
    leftBase.SetPercent(10);
    rightBase.SetPercent(10);

    Sleep(100);

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
    AnalogInputPin cds(FEHIO::P3_0);

    armServo.SetMin(748);
    armServo.SetMax(2500);

    float x, y;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    // Wait for start light or 30 seconds
    LCD.WriteLine("I'm waiting :P");
    float startTime = TimeNow();
    while((TimeNow() - startTime < 30) && cds.Value() > RED_THRESHOLD) {
        Sleep(100);
    }
    LCD.WriteLine("Sigh...");

    // Set servo to initial position
    armServo.SetDegree(90);

    // To ramp
    autoDriveF(6);
    Sleep(250);

    autoTurnR(2.75);
    Sleep(250);

    autoDriveF(14);
    Sleep(500);

    autoTurnL(6.4);
    Sleep(250);

    // Up ramp to foosball
    LCD.WriteLine("I'm scared ;-;");
    upRamp();
    Sleep(250);
    LCD.WriteLine("Am I still alive?");

    autoDriveF(10);
    Sleep(250);

    autoTurnR(5.75);
    Sleep(250);

    timeDrive(25, 1500);

    // Backwards to lever
    autoDriveB(20);
    Sleep(250);

    armServo.SetDegree(40);
    Sleep(500);

    autoDriveB(1);
    Sleep(250);

    leftBase.SetPercent(-50);
    rightBase.SetPercent(-50);
    Sleep(250);
    leftBase.SetPercent(0);
    rightBase.SetPercent(0);
    armServo.SetDegree(90);
    autoTurnL(0.75);
    Sleep(250);

    autoDriveB(4);
    Sleep(250);

    // To ramp
    autoTurnL(2.75);
    Sleep(250);

    LCD.WriteLine("Fingers crossed");
    Sleep(500);
    LCD.WriteLine("DEEP BREATHS");
    Sleep(500);
    LCD.Write("3...");
    Sleep(500);
    LCD.WriteLine("2...");
    Sleep(500);
    LCD.WriteLine("Yolo");

    autoDriveB(60);

    // Boom
    autoTurnR(2.75);
    timeDrive(50, 1000);

    /*
    while(1) {
        while(!LCD.Touch(&x, &y)) {
            LCD.WriteLine(cds.Value());
            Sleep(250);
        }
        armServo.SetDegree(0);
        Sleep(1000);
        while(!LCD.Touch(&x, &y)) {
            LCD.WriteLine(cds.Value());
            Sleep(250);
        }
        armServo.SetDegree(30);
        Sleep(1000);
    }
    */

    /*
    while(1) {
        while(!LCD.Touch(&x, &y));
        autoDriveF(24);
        while(!LCD.Touch(&x, &y));
        autoDriveB(24);
        while(!LCD.Touch(&x, &y));
        autoTurnL(12);
        while(!LCD.Touch(&x, &y));
        autoTurnR(12);
    }
    */

    return 0;
}
