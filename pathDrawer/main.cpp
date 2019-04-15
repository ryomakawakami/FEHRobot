#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <stdlib.h>
#include <cmath>

#define TICKS_PER_INCH 28
#define INCH_PER_PIXEL 0.2
#define RADIUS 3
#define MAXIMUM_POINTS 100
#define SKIP_SOME 4
#define PI 3.1415926536

using namespace std;

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

    FEHMotor leftBase(FEHMotor::Motor0, 9);
    FEHMotor rightBase(FEHMotor::Motor1, 9);

    DigitalEncoder rightEnc(FEHIO::P0_0);
    DigitalEncoder leftEnc(FEHIO::P1_0);

    bool done = false, doneDrawing = false;

    int index = 0;

    int counter = 0;

    while (!done) {

        int* xPosition;
        int* yPosition;
        int* lCounts = (int*) calloc(MAXIMUM_POINTS - 1, sizeof(int));
        int* rCounts = (int*) calloc(MAXIMUM_POINTS - 1, sizeof(int));

        while (!doneDrawing) {
            xPosition = (int*) calloc(MAXIMUM_POINTS, sizeof(int));
            yPosition = (int*) calloc(MAXIMUM_POINTS, sizeof(int));

            LCD.Clear(FEHLCD::Black);
            displayMenu();

            while (!LCD.Touch(&x, &y));
            while (LCD.Touch(&x, &y)) {
                if (x > 241) {
                    x = 241;
                }
                if (y > 180) {
                    y = 180;
                }

                if (index < MAXIMUM_POINTS) {
                    LCD.DrawPixel(x, y);

                    counter++;
                    if (counter >= SKIP_SOME) {
                        if (x != *(xPosition + index) || y != *(yPosition + index)) {
                            counter = 0;
                            *(xPosition + index) = x;
                            *(yPosition + index) = y;
                            index++;
                        }
                    }
                }

                Sleep(10);
            }

            displayOptions();

            while (!LCD.Touch(&x, &y));
            if (y < 160) {
                doneDrawing = true;
            }
        }
        LCD.Clear(FEHLCD::Black);

        double lastTheta = 0;

        for (int i = 0; i < index - 1; i++) {
            int deltaX = *(xPosition + i + 1) - *(xPosition + i);
            int deltaY = *(yPosition + i + 1) - *(yPosition + i);
            double r = sqrt(deltaX * deltaX + deltaY * deltaY);

            double theta = 0;
            if (y >= 0) {
                if (y == 0) {
                    if (x == 0) {
                        theta = 0;
                    }
                    else if (x > 0) {
                        theta = PI / 2;
                    }
                    else {
                        theta = - PI / 2;
                    }
                }
                else {
                    theta = atan(((double) deltaX) / deltaY);
                }
            }
            else {
                theta = atan(((double) deltaX) / deltaY) + PI;
            }

            double dist = r * INCH_PER_PIXEL * TICKS_PER_INCH;
            double diff = RADIUS * TICKS_PER_INCH * (theta - lastTheta);

            lastTheta = theta;

            *(lCounts + i) = dist - diff;
            *(rCounts + i) = dist + diff;
        }

        for (int i = 0; i < index - 1; i++) {
            leftEnc.ResetCounts();
            rightEnc.ResetCounts();

            bool leftDone = false, rightDone = false;
            int leftTarget, rightTarget;

            if (*(lCounts + i) < 0) {
                leftTarget = -*(lCounts + i);
                leftBase.SetPercent(15);
            }
            else {
                leftTarget = *(lCounts + i);
                leftBase.SetPercent(-15);
            }

            if (*(rCounts + i) < 0) {
                rightTarget = -*(rCounts + i);
                rightBase.SetPercent(-15);
            }
            else {
                rightTarget = *(rCounts + i);
                rightBase.SetPercent(15);
            }

            while (!leftDone || !rightDone) {
                if (leftEnc.Counts() > leftTarget) {
                    leftBase.SetPercent(0);
                    leftDone = true;
                }
                if (rightEnc.Counts() > rightTarget) {
                    rightBase.SetPercent(0);
                    rightDone = true;
                }
            }

            Sleep(10);

        }

        free(xPosition);
        free(yPosition);
        free(lCounts);
        free(rCounts);

        index = 0;
        counter = 0;

        doneDrawing = false;
    }

    return 0;
}
