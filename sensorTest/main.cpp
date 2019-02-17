#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>

DigitalEncoder enc(FEHIO::P0_0);

int main(void)
{

    float x,y;

    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    while( true )
    {
        LCD.WriteLine(enc.Counts());

        Sleep(50);
    }
    return 0;
}
