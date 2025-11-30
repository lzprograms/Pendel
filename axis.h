#ifndef AXIS_H
#define AXIS_H

#include <gpiod.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <queue>


class Axis { 
private:
    enum class MotorState {
        POS,                // accelerate in direction of position
        SPEED,              // accelerate until speed is reached
        BRAKEPOS, 
        BRAKESPEED, 
        HOME, 
        CHANGEDIRECTION
        };
    
    struct MotorTask {
        MotorState state;
        int arg; // desired Speed or desired Position otherwise 0
    };
    
    std::queue<MotorTask> tasks;


    bool running;                   // Läuft Thread?
    std::thread thread;             // Thread für Motorposition
    
    int pos;                // current position in steps
    int endPos;             // total amount of steps from one side to the other
    int direction;          // current direction


    int curSpeed;           // current speed in motor steps per second
    int maxSpeed = 800000;           // max speed in motor steps per second
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
    void stopThread();

    // Bewegungssteuerung
    bool setPos(int newPos);
    bool setSpeed(int stepsPerSecond);
    bool home();

    // Getter
    double getEndPos();
    int getPos();
};

#endif // AXIS_H

