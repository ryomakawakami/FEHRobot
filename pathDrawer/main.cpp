#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>

void displayMenu() {
    LCD.DrawRectangle(1, 0, 240, 180);
}

void displayOptions() {
    LCD.WriteAt("Okay", 260, 50);
    LCD.WriteAt("Oops", 260, 170);
}

int main(void) {

    float x, y;

    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    bool done = false, doneDrawing = false;
    //vector<int> xCoord;
    //vector<int> yCoord;

    while (!done) {
        while (!doneDrawing) {
            LCD.Clear(FEHLCD::Black);
            displayMenu();

            while (!LCD.Touch(&x, &y));
            while (LCD.Touch(&x, &y)) {
                if (x > 240) {
                    x = 240;
                }
                if (y > 180) {
                    y = 180;
                }
                LCD.DrawPixel(x, y);
            }

            displayOptions();

            while (!LCD.Touch(&x, &y));
            if (y < 160) {
                doneDrawing = true;
            }
        }
        LCD.Clear(FEHLCD::Black);
    }

    return 0;
}
