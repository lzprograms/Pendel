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

public:
    Pendel();
    ~Pendel();

    double getWinkelGrad() const;  // in Grad
    int getAngle() const; // in Anzahl Drehgeberimpulse
    double getAngleVelocity() const;
    double getPos() const;      // in mm
    int getPosSchritte() const; // in Motorschritten
    int getAchseLaenge() const;
    bool setPos(double pos);
    bool setRelPos(double pos);
    void calibratePos();
    void consoleOut() const;
};

#endif // PENDEL_H
