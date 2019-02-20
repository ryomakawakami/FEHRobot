#include <FEHLCD.h>
#include <FEHIO_quad.h>
#include <FEHUtility.h>

int main(void)
{
    float x,y;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    // How do I include FEHIO_quad in the build process
    QuadEncoder enc(FEHIO::P0_0, FEHIO::P0_1);

    while(1) {

    }

    return 0;
}
