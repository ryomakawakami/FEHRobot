#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHAccel.h>
#include <cmath>

// Gravity in px / s^2
#define A_G 1000

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
    Ball(double s, double k, double x, double y);
    double getX();
    double getY();
    void setX(double x);
    void setY(double y);
    void updateAccel(double x, double y);
private:
    double mu_s, mu_k;
    double r_x, r_y;
    double v_x, v_y;
    double a_x, a_y;
    float lastTime;
    void update();
};

Ball::Ball(double s, double k, double x, double y) {
    mu_s = s;
    mu_k = k;
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

    /* FRICTION STILL BREAKS EVERYTHING (IT'S PROBABLY JUST ABOUT TUNING MU THOUGH)
    // Terrible approximation of friction (N is always mg despite being tilted)
    // TODO: Figure out the actual normal force

    // Static friction
    if (v_x * v_x + v_y * v_y == 0) {
        if (fabs(a_x * a_x + a_y * a_y) < (mu_s * A_G)) {
            a_x = 0;
            a_y = 0;
        }
    }

    // Kinetic friction
    else {
        double theta = atan(v_y / v_x);      // Safe in theory since v_x != 0
        double friction = mu_k * A_G;
        if (v_x * v_x + v_y * v_y > 0) {
            a_x -= friction * cos(theta);
            a_y -= friction * sin(theta);
        }
        else {
            a_x += friction * cos(theta);
            a_y += friction * sin(theta);
        }
    }
    */

    r_x += v_x * deltaTime;
    r_y += v_y * deltaTime;

    v_x += a_x * deltaTime;
    v_y += a_y * deltaTime;
}

int main(void) {
    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    float x, y;

    Ball ball(0.1, 0.1, (X_MAX - X_MIN) / 2, (Y_MAX - Y_MIN) / 2);

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
