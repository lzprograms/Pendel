#ifndef ENCODER_H
#define ENCODER_H

#include <gpiod.h>
#include <thread>
#include <cmath>
#include <iostream>

class Encoder {
private:
    int winkel;                    // Aktueller Winkel in Sensorschritten
    int winkelSchritte;            // Anzahl der Schritte pro Umdrehung
    double schrittDist;            // Distanz pro Schritt

    bool running;                  // Läuft der Thread?
    std::thread thread;            // Thread zur Winkelüberwachung

    gpiod_line* a_line;            // Phase A
    gpiod_line* b_line;            // Phase B

    // Thread-Funktion zur Ereignisüberwachung
    void eventLoop();

    // Thread starten
    bool startThread();

    // Thread stoppen
    void stopThread();

public:
    // Konstruktor
    Encoder(gpiod_chip* chip, unsigned int pinA, unsigned int pinB,
            int winkelSchritte = 640, double schrittDist = 0.2);

    // Destruktor
    ~Encoder();

    // Liefert den aktuellen Winkel in Grad zurück
    double getWinkelGrad() const;
};

#endif // ENCODER_H

