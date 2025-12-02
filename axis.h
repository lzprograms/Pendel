#ifndef AXIS_H
#define AXIS_H

#include <gpiod.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "SafeQueue.h"


class Axis { 
private:
    enum class MotorState {
        POS,                // accelerate in direction of position
        SPEED,              // accelerate until speed is reached
        BRAKEPOS, 
        BRAKESPEED, 
        CHANGEDIRECTION,
        FINISHHOME
        };
    
    struct MotorTask {
        MotorState state;
        int arg; // desired Speed or desired Position otherwise 0
    };
    
    SafeQueue<MotorTask> tasks;


    bool running;                   // Läuft Thread?
    std::thread thread;             // Thread für Motorposition
    bool isHoming;
    
    int pos;                // current position in steps
    int endPos;             // total amount of steps from one side to the other
    int direction;          // current direction


    int curSpeed;           // current speed in motor steps per second
    int maxSpeed = 75000;           // max speed in motor steps per second
    int maxAcceleration = 100000;    // maximum acceleration in steps/second*second
    double inverseMaxSpeed;
    int usDelay;
    int waitedUs;
    int minUsDelay;
    double accumulatedAcceleration;

    gpiod_line* step_line;
    gpiod_line* dir_line;
    gpiod_line* en_line;
    gpiod_line* endschalter_line;
    
    gpiod_line* ms1_line;
    gpiod_line* ms2_line;

    // --- internal loop ---
    void eventLoop();
    
    void clearQueue();
    void stopAndReverse(int direction);

    void maxAccelerate(int untilSpeed);
    void maxDecelerate(int untilSpeed = 0);
    void decelerateForPos(int remainingDistance);
    void accelerate(double value);
    void stopOnLimit();
    int getDecelDistance();

public:
    // Konstruktor / Destruktor
    Axis(gpiod_chip* chip,
         unsigned int pinStep,
         unsigned int pinDir,
         unsigned int pinEn,
         unsigned int pinEndschalter,
         unsigned int pinMs1,
         unsigned int pinMs2,
         int achseLaengeP);

    ~Axis();

    // Threadsteuerung
    bool startThread();
    bool isCalibrating();
    void stopThread();

    // Bewegungssteuerung
    bool setPos(int newPos);
    bool setSpeed(int stepsPerSecond);
    bool home();
    
    
    
    bool setMaxSpeed(int stepsPerSecond);
    bool setMaxAcceleration(int stepsPerSecond2);

    // Getter
    double getEndPos();
    int getPos();
    int getSpeed();
};

#endif // AXIS_H

