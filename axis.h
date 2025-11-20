#ifndef AXIS_H
#define AXIS_H

#include <gpiod.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <stdexcept>


class Axis { 
private:
    bool running;                   // Läuft Thread?
    std::thread thread;             // Thread für Motorposition
    
    double pos;                     // Ist-Position des Schlittens
    double sollPos;                 // Soll-Position des Schlittens
    double achseLaenge;             // Gesamtstrecke in mm
    double schrittDist;             // Distanz in mm pro Motorschritt

    const int usDelay1 = 100;       // Zeit für Step-Puls (µs)
    const int usDelay2 = 100;       // Zeit zwischen Step-Pulsen

    gpiod_line* step_line;
    gpiod_line* dir_line;
    gpiod_line* en_line;
    gpiod_line* endschalter_line;
    
    gpiod_line* ms1_line;
    gpiod_line* ms2_line;

    // --- Interner Steuerloop ---
    void eventLoop();

public:
    // Konstruktor / Destruktor
    Axis(gpiod_chip* chip,
         unsigned int pinStep,
         unsigned int pinDir,
         unsigned int pinEn,
         unsigned int pinEndschalter,
         unsigned int pinMs1,
         unsigned int pinMs2,
         int achseLaengeP,
         double schrittDistP);

    ~Axis();

    // Threadsteuerung
    bool startThread();
    void stopThread();

    // Bewegungssteuerung
    bool setPos(double newPos);
    bool home();

    // Getter
    double getAchseLaenge();
    double getPos();
};

#endif // AXIS_H

