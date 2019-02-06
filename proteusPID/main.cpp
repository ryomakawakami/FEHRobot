#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHAccel.h>
#include <FEHMotor.h>
#include <pidlib.h>

int main(void)
{
    FEHMotor motor(FEHMotor::Motor0,6);

    PID pid(100, 0, 0, 0);

    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    double output;

    while( true )
    {
        output = pid.calculate(0, Accel.X());

        motor.SetPercent(output);

        LCD.WriteLine(output);

        Sleep(20);
    }
    return 0;
}
