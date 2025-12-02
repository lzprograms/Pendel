#ifndef PENDEL_H
#define PENDEL_H

#include <gpiod.h>
#include <iostream>
#include <stdexcept>
#include "encoder.h"
#include "axis.h"


class Pendel {
private:
    Encoder* encoder;
    Axis* axis;
    const char* chipname = "gpiochip0"; // Fest für Raspberry Pi 3
    gpiod_chip* chip;
    bool plotRunning;
        std::thread thread;             // Thread for plot stream

public:
    Pendel();
    ~Pendel();

    double getWinkelGrad() const;  // in Grad
    int getAngle() const; // in Anzahl Drehgeberimpulse
    int getPos() const;      // in mm
    int getAngleVelocity() const;
    int getEndPos() const;
    bool setPos(double pos);
    bool setRelPos(double pos);
    bool setSpeed(int speed);
    void calibratePos();
    bool isCalibrating();
    bool setMaxSpeed(int stepsPerSecond);
    bool setMaxAcceleration(int stepsPerSecond2);
    void consoleOut() const;
    void plotLoop() const;
    void startPlot();
};

#endif // PENDEL_H
