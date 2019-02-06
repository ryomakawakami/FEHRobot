#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHServo.h>
#include <FEHAccel.h>

int main(void)
{

    FEHServo servo(FEHServo::Servo0);

    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    while( true )
    {
        LCD.Clear (FEHLCD::Black);

        LCD.WriteLine(Accel.X());

        servo.SetDegree(Accel.X()*90 + 90);

        Sleep(100);
    }
    return 0;
}
