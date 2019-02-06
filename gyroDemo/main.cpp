#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHAccel.h>
#include <cmath>

// Gravity in px / s^2
#define A_G 1000

// Coefficients of friction
#define MU_S 1
#define MU_K 0.0001

// Screen size
#define X_MIN 1
#define X_MAX 319
#define Y_MIN 0
#define Y_MAX 238

using namespace std;

const double pi = 3.1415926536;

double degToRad(double deg) {
    return deg * pi / 180.;
}

class Ball {
public:
    Ball(double x, double y);
    double getX();
    double getY();
    void setX(double x);
    void setY(double y);
    void updateAccel(double x, double y);
private:
    double r_x, r_y;
    double v_x, v_y;
    double a_x, a_y;
    float lastTime;
    void update();
};

Ball::Ball(double x, double y) {
    r_x = x;
    r_y = y;
    v_x = 0;
    v_y = 0;
    a_x = 0;
    a_y = 0;
    lastTime = TimeNow();
}

double Ball::getX() {
    return r_x;
}

double Ball::getY() {
    return r_y;
}

void Ball::setX(double x) {
    r_x = x;
    v_x = 0;
    a_x = 0;
}

void Ball::setY(double y) {
    r_y = y;
    v_y = 0;
    a_y = 0;
}

void Ball::updateAccel(double x, double y) {
    a_x = x;
    a_y = y;

    update();

    lastTime = TimeNow();
}

void Ball::update() {
    float deltaTime = TimeNow() - lastTime;

    r_x += v_x * deltaTime;
    r_y += v_y * deltaTime;

    v_x += a_x * deltaTime;
    v_y += a_y * deltaTime;

    /* FRICTION BREAKS EVERYTHING
    // Friction loss in velocity = sqrt(2 * mu * g * d * cos theta)
    // Doesn't consider the angle of the Proteus but it should
    double frictionX = sqrt(2 * MU_K * A_G * v_x * deltaTime);
    double frictionY = sqrt(2 * MU_K * A_G * v_y * deltaTime);

    if (fabs(v_x) < frictionX)
        v_x = 0;
    else if (v_x > 0)
        v_x -= frictionX;
    else
        v_x += frictionX;

    if (fabs(v_y) < frictionY)
        v_y = 0;
    else if (v_y > 0)
        v_y -= frictionY;
    else
        v_y += frictionY;*/
}

int main(void) {
    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    float x, y;

    Ball ball((X_MAX - X_MIN) / 2, (Y_MAX - Y_MIN) / 2);

    while(true) {
        ball.updateAccel(A_G * sin(degToRad(Accel.X())), -A_G * sin(degToRad(Accel.Y())));

        if (ball.getX() < X_MIN)
            ball.setX(X_MIN);

        else if (ball.getX() > X_MAX)
            ball.setX(X_MAX);

        if (ball.getY() < Y_MIN)
            ball.setY(Y_MIN);

        else if (ball.getY() > Y_MAX)
            ball.setY(Y_MAX);

        //LCD.Clear(FEHLCD::Black);
        LCD.DrawPixel((int) ball.getX(), (int) ball.getY());

        Sleep(100);
    }

    return 0;
}
