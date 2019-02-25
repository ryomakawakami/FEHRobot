#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHAccel.h>
#include <FEHSD.h>

// Declare motors
FEHMotor leftBase(FEHMotor::Motor0, 9);
FEHMotor rightBase(FEHMotor::Motor1, 9);

// Declare encoders
DigitalEncoder leftEnc(FEHIO::P0_1);
DigitalEncoder rightEnc(FEHIO::P1_0);

int main(void)
{
    float x,y;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    // Loop through all power settings
    for(int i = 55; i <= 100; i++) {
        // Holds velocity
        double velocity;

        // Create new log
        SD.OpenLog();

        // Reset encoder counts
        leftEnc.ResetCounts();
        rightEnc.ResetCounts();

        // Wait for 2 seconds
        LCD.Write("Ready for ");
        LCD.WriteLine(i);
        LCD.WriteLine("");
        Sleep(2000);

        // Accelerate for 0.25 second
        for(int j = 1; j <= 5; j++) {
            leftBase.SetPercent(i * j / 5);
            rightBase.SetPercent(- i * j / 5);
            Sleep(50);
        }

        // Run for 1 second
        float startTime = TimeNow();
        while(TimeNow() - startTime < 1) {
            SD.Printf("%f %d %d\n", TimeNow(), leftEnc.Counts(), rightEnc.Counts());
            Sleep(10);
        }

        // Stop base
        leftBase.SetPercent(0);
        rightBase.SetPercent(0);

        // Close log
        SD.CloseLog();
    }

    return 0;
}
