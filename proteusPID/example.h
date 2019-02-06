#define MAXSTEP 10  // Max change per iteration
#define LOOPTIME 20   // 20 ms, 50 Hz

#include "pidlib.h"

// PID objects with somewhat reasonable constants
// Only P is used
// Not sure if constructors work in this scope
PID basePID(0.5, 0, 0, 0), driftPID(1, 0, 0, 0);

// Sets left base motors
void driveL(int power) {
	motor[left] = power;
}

// Sets right base motors
void driveR(int power) {
	motor[right] = power;
}

// Returns left encoder value
int getLeftEncoder() {
	return SensorValue[leftEncoder];
}

// Returns right encoder value
int getRightEncoder() {
	return SensorValue[rightEncoder];
}

// Clears encoders
void clearEncoders() {
	SensorValue[leftEncoder] = 0;
	SensorValue[rightEncoder] = 0;
}

// Using only left encoder
void autoDrive(int target) {

	bool done = false;
	long closeTime;
	float driveOut;

	clearEncoders();

	while(!done) {
		
		// Calculate motor output
		driveOut = basePID.calculate(target, getLeftEncoder());
		
		// Set motors to output
		driveL(driveOut);
		driveR(driveOut);

		// Sleep for set time
		sleep(LOOPSPEED);

		// Loop ends .25 seconds after left encoder gets within 100 ticks
		if(abs(target - getLeftEncoder()) > 100) {
			closeTime = getTime();
		}

		if((getTime() - closeTime) > 250) {
			done = true;
		}

	}

	// Stop motors
	driveL(0);
	driveR(0);

}

// Using both encoders to drive straight
void autoDrive2(int target) {

	bool done = false;
	long closeTime;
	float driveOut, driftOut;

	clearEncoders();

	while(!done) {
		
		// Calculate motor output
		driveOut = basePID.calculate(target, (getLeftEncoder()+getRightEncoder())/2);
		driftOut = driftPID.calculate(target, getLeftEncoder() - getRightEncoder());

		// Limit driveOut from -100 to 100
		if(abs(driveOut) > 100) {
			driveOut = sgn(driveOut) * 100;
		}

		// Set motors to output
		driveL(driveOut - driftOut);
		driveR(driveOut + driftOut);

		// Sleep for set time
		sleep(LOOPSPEED);

		// Loop ends .25 seconds after average gets within 100 ticks
		if(abs(target - (getLeftEncoder()+getRightEncoder())/2) > 100) {
			closeTime = getTime();
		}

		if((getTime() - closeTime) > 250) {
			done = true;
		}

	}

	// Stop motors
	driveL(0);
	driveR(0);

}

// Using both encoders and slew rate control
void autoDrive3(int target) {

	bool done = false;
	long closeTime;
	float driveOut, driftOut;
	float outL, outR, lastOutL, lastOutR;

	clearEncoders();

	while(!done) {
		
		// Calculate motor output
		driveOut = basePID.calculate(target, (getLeftEncoder()+getRightEncoder())/2);
		driftOut = driftPID.calculate(target, getLeftEncoder() - getRightEncoder());

		// Limit driveOut from -100 to 100
		if(abs(driveOut) > 100) {
			driveOut = sgn(driveOut) * 100;
		}

		// Calculate motor output
		outL = driveOut - driftOut;
		outR = driveOut + driftOut;

		// Slew rate limit
		if(outL - lastOutL > MAXSTEP) {
			outL = lastOutL + 10;
		}
		else if(outL - lastOutL < -MAXSTEP) {
			outL = lastOutL - 10;
		}

		if(outR - lastOutR > MAXSTEP) {
			outR = lastOutR + 10;
		}
		else if(outR - lastOutR < -MAXSTEP) {
			outR = lastOutR - 10;
		}

		// Set motors to output
		driveL(outL);
		driveR(outR);

		// Store output for slew rate
		lastOutL = outL;
		lastOutR = outR;

		// Sleep for set time
		sleep(LOOPSPEED);

		// Loop ends .25 seconds after average gets within 100 ticks
		if(abs(target - (getLeftEncoder()+getRightEncoder())/2) > 100) {
			closeTime = getTime();
		}

		if((getTime() - closeTime) > 250) {
			done = true;
		}
	}

	// Stop motors
	driveL(0);
	driveR(0);
	
}

void pre_auton() {

}