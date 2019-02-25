#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include "colors.h"

#define background 0xCECCD1

void drawPicture(int colors[], int size_x, int size_y, int pos_x, int pos_y) {
    for(int j = 0; j < size_y; j++) {
        for(int i = 0; i < size_x; i++) {
            LCD.SetFontColor(colors[i+j*size_x]);
            LCD.DrawPixel(i + pos_x, j + pos_y);
            /*
            LCD.DrawPixel(i*2, j*2);
            LCD.DrawPixel(i*2-1, j*2);
            LCD.DrawPixel(i*2, j*2-1);
            LCD.DrawPixel(i*2-1, j*2-1);
            */
            /*
            LCD.DrawPixel(i, j+18);
            LCD.DrawPixel(i+111, j+18);
            LCD.DrawPixel(i+222, j+18);
            LCD.DrawPixel(i, j+137);
            LCD.DrawPixel(i+111, j+137);
            LCD.DrawPixel(i+222, j+137);
            */
        }
    }
}

void drawPicture2(int colors[], int size_x, int size_y, int pos_x, int pos_y) {
    for(int j = 0; j < size_y; j++) {
        for(int i = 0; i < size_x; i++) {
            LCD.SetFontColor(colors[i+j*size_x]);
            LCD.DrawPixel(i*2+pos_x, j*2+pos_y);
            LCD.DrawPixel(i*2-1+pos_x, j*2+pos_y);
            LCD.DrawPixel(i*2+pos_x, j*2-1+pos_y);
            LCD.DrawPixel(i*2-1+pos_x, j*2-1+pos_y);
        }
    }
}

int main(void)
{

    float x,y;

    LCD.Clear(background);
    LCD.SetFontColor(FEHLCD::White);

    drawPicture2(pic1, 49, 41, 5, 5);
    drawPicture(pic2, 96, 96, 175, 100);

    while( true )
    {

    }
    return 0;
}
