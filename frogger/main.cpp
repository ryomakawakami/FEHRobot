#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHAccel.h>

#define NUMBER_OF_COLORS 8
#define COLOR_RED FEHLCD::Red
#define COLOR_ORANGE 0xFFA500
#define COLOR_YELLOW 0xFFFF00
#define COLOR_GREEN 0x008000
#define COLOR_BLUE FEHLCD::Blue
#define COLOR_PURPLE 0x800080
#define COLOR_BLACK FEHLCD::Black
#define COLOR_WHITE FEHLCD::White

enum {
    _RED,
    _ORANGE,
    _YELLOW,
    _GREEN,
    _BLUE,
    _PURPLE,
    _BLACK,
    _WHITE
};

void setColor(int color) {
    switch (color) {
        case _RED:
            LCD.SetFontColor(COLOR_RED);
        break;
        case _ORANGE:
            LCD.SetFontColor(COLOR_ORANGE);
        break;
        case _YELLOW:
            LCD.SetFontColor(COLOR_YELLOW);
        break;
        case _GREEN:
            LCD.SetFontColor(COLOR_GREEN);
        break;
        case _BLUE:
            LCD.SetFontColor(COLOR_BLUE);
        break;
        case _PURPLE:
            LCD.SetFontColor(COLOR_PURPLE);
        break;
        case _BLACK:
            LCD.SetFontColor(COLOR_BLACK);
        break;
        case _WHITE:
        default:
            LCD.SetFontColor(COLOR_WHITE);
        break;
    }
}

void drawColorPalette() {
    LCD.SetFontColor(COLOR_RED);
    LCD.DrawRectangle(1, 200, 38, 39);
    LCD.SetFontColor(COLOR_ORANGE);
    LCD.DrawRectangle(40, 200, 39, 39);
    LCD.SetFontColor(COLOR_YELLOW);
    LCD.DrawRectangle(80, 200, 39, 39);
    LCD.SetFontColor(COLOR_GREEN);
    LCD.DrawRectangle(120, 200, 39, 39);
    LCD.SetFontColor(COLOR_BLUE);
    LCD.DrawRectangle(160, 200, 39, 39);
    LCD.SetFontColor(COLOR_PURPLE);
    LCD.DrawRectangle(200, 200, 39, 39);
    LCD.SetFontColor(COLOR_BLACK);
    LCD.DrawRectangle(240, 200, 39, 39);
    LCD.SetFontColor(COLOR_WHITE);
    LCD.DrawRectangle(280, 200, 38, 39);
}

int main(void) {

    float x, y;

    LCD.Clear(COLOR_BLACK);
    LCD.SetFontColor(COLOR_WHITE);

    bool quit = false;

    drawColorPalette();

    while(!quit) {
        if (LCD.Touch(&x, &y)) {
            if (y < 200) {
                LCD.DrawPixel(x, y);
            }
            else {
                int color = x / 40;
                LCD.SetFontColor(COLOR_WHITE);
                for (int i = 0; i < NUMBER_OF_COLORS; i++) {
                    if (i != color) {
                        LCD.WriteAt(" ", i * 40 + 12, 210);
                    }
                    else {
                        LCD.WriteAt("o", i * 40 + 12, 210);
                    }
                }
                setColor(color);
            }
        }
        if (Accel.Y() > 0.75) {
            LCD.Clear(FEHLCD::Black);
            drawColorPalette();
            while (Accel.Y() > 0.75);
        }
    }
    return 0;
}
