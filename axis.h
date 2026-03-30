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
        POS,                // go to position as fast as possible
        SPEED,              // accelerate until speed is reached
        ACCELERATE,         // accelerate
        BRAKEPOS, 
        BRAKESPEED, 
        CHANGEDIRECTION
        };
    
    struct MotorTask {
        MotorState state;
        int arg; // desired Speed or desired Position otherwise 0
    };
    
    SafeQueue<MotorTask> tasks;

    bool running;                   // LÃ¤uft Thread?
    std::thread thread;             // Thread fÃ¼r Motorposition
    bool isHoming;
    int lastRightLimitValue = 1; // start, 1 = not pressed (pull up) 
    int lastLeftLimitValue = 1; // start, not pressed
    
    int pos;                // current position in steps
    int endPos;             // total amount of steps from one side to the other
    int direction;          // current direction


    int curSpeed;           // current speed in motor steps per second
    int maxSpeed = 120000;           // max speed in motor steps per second
    int maxAcceleration = 120000;    // maximum acceleration in steps/second*second
    double inverseMaxSpeed;
    int usDelay;
    int waitedUs;
    int minUsDelay;
    double accumulatedAcceleration;

    gpiod_line* step_line;
    gpiod_line* dir_line;
    gpiod_line* en_line;
    gpiod_line* right_limit_line;
    gpiod_line* left_limit_line;
    
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
    void checkLimits();
    int getDecelDistance();

public:
    // Konstruktor / Destruktor
    Axis(gpiod_chip* chip,
         unsigned int pinStep,
         unsigned int pinDir,
         unsigned int pinEn,
         unsigned int pinRightLimit,
         unsigned int pinLeftLimit,
         unsigned int pinMs1,
         unsigned int pinMs2
         );

    ~Axis();

    // Threadsteuerung
    bool startThread();
    bool isCalibrating();
    
    void stopThread();

    // Bewegungssteuerung
    bool setPos(int newPos);
    bool setSpeed(int stepsPerSecond);
    bool setAcceleration(int stepsPerSecond);
    bool home();
    
    
    
    bool setMaxSpeed(int stepsPerSecond);
    bool setMaxAcceleration(int stepsPerSecond2);

    // Getter
    double getEndPos();
    int getPos();
    int getSpeed();
};



