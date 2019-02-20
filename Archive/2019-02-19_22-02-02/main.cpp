#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <pidlib.h>

#define MAX_STEP 10  // Max change per iteration
#define LOOP_TIME 0.020   // 20 ms, 50 Hz

#define MAX_VELOCITY 28 // Max (reasonable) velocity in in/s
#define VELOCITY_RANGE 12 // Range of velocity PID in in

#define TICKS_PER_INCH 39 // Conversion from encoder ticks to in

// Declare servo
FEHServo armServo(FEHServo::Servo0);

// Declare motors
FEHMotor leftBase(FEHMotor::Motor0, 9);
FEHMotor rightBase(FEHMotor::Motor1, 9);

// Declare encoders
DigitalEncoder leftEnc(FEHIO::P0_0);
DigitalEncoder rightEnc(FEHIO::P1_0);

// PID control loop
// target is desired encoder count, vTarget is desired velocity, vRange is active range of velocity PID
// Velocity PID until some distance to target
// Position PID when some distance away
// Drift PID and slew rate are constantly active
// Ends function 250 ms after gets close
// MAX_STEP is slew rate limit (10%)
// LOOP_TIME is time per update (20 ms)
void autoDrive(float target, float vTarget = MAX_VELOCITY, float vRange = VELOCITY_RANGE) {
    PID basePID(0.4, 0.001, 0, 0), velocityPID(0.01, 0, 0, 0), driftPID(1, 0, 0, 0);

    bool done = false;
    float closeTime;
    float driveOut, driftOut;
    float outL, outR, lastOutL = 0, lastOutR = 0;
    float avgEnc, currentTime;
    float velocity, lastAvgEnc = 0, lastTime = TimeNow() - LOOP_TIME;

    target *= TICKS_PER_INCH;
    vTarget *= TICKS_PER_INCH;
    vRange *= TICKS_PER_INCH * target / fabs(target);

    basePID.initialize();
    velocityPID.initialize();
    driftPID.initialize();

    // Consider allowing for accumulating error
    leftEnc.ResetCounts();
    rightEnc.ResetCounts();

    while(!done) {
        // Update current time
        currentTime = TimeNow();

        // Update average distance
        avgEnc = (leftEnc.Counts() + rightEnc.Counts()) / 2;

        // Calculate velocity
        velocity = (avgEnc - lastAvgEnc) / (currentTime - lastTime);
        lastAvgEnc = avgEnc;
        lastTime = currentTime;

        // Drift PID
        driftOut = driftPID.calculate(0, leftEnc.Counts() - rightEnc.Counts());

        // Velocity PID
        if(fabs(target) - fabs(avgEnc) > vRange) {
            driveOut = velocityPID.calculate(vTarget, velocity);

            // Calculate motor outputs
            // Limit driveOut contribution so driftOut can have affect it?
            outL += driveOut;
            outR += driveOut;

            LCD.WriteLine(driveOut);
        }

        // Position PID
        else {
            driveOut = basePID.calculate(target, avgEnc);

            // Calculate motor outputs
            // Limit driveOut contribution so driftOut can have affect it?
            outL = driveOut;
            outR = driveOut;
        }

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

// PID control loop without velocity control
// target is desired encoder count
// Position PID when some distance away
// Drift PID and slew rate are constantly active
// Ends function once at location
// MAX_STEP is slew rate limit (10%)
// LOOP_TIME is time per update (20 ms)
void autoDriveIncremental(float target) {
    PID basePID(0.4, 0.001, 0, 0), driftPID(1, 0, 0, 0);

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
    leftBase.SetPercent(0);
    rightBase.SetPercent(0);
}

int main(void)
{
    AnalogInputPin cds(FEHIO::P3_0);

    armServo.SetMin(748);
    armServo.SetMax(2500);

    float x, y;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    while(1) {
        LCD.WriteLine(cds.Value());
        Sleep(100);
    }

    /*
    while(1) {
        while(!LCD.Touch(&x, &y));
        armServo.SetDegree(0);
        Sleep(1000);
        while(!LCD.Touch(&x, &y));
        armServo.SetDegree(30);
        Sleep(1000);
    }
    */

    /*
    int oldL, oldR;

    leftBase.SetPercent(100);
    rightBase.SetPercent(100);

    while(1) {
        oldL = leftEnc.Counts();
        oldR = rightEnc.Counts();

        Sleep(100);

        LCD.Write(leftEnc.Counts());
        LCD.Write(" ");
        LCD.Write(leftEnc.Counts() - oldL);
        LCD.Write(" ");
        LCD.Write(rightEnc.Counts() - oldR);
        LCD.Write(" ");
        LCD.WriteLine(oldR);
    }
    */

    /*
    while(1) {
        while(!LCD.Touch(&x, &y));
        autoDriveIncremental(48);
    }
    */

    return 0;
}
