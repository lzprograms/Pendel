#ifndef ENCODER_H
#define ENCODER_H

#include <gpiod.h>
#include <thread>
#include <cmath>
#include <iostream>
#include <vector>
#include <poll.h>
#include <queue>

struct EdgeLine {
    char line;			 //Flanke 'A' oder 'B'
    gpiod_line_event event;      // Original-Event
};

bool isEarlier(const timespec &a, const timespec &b);//Hilfsfunktion
int timeApartNS(const timespec &a, const timespec &b);

class Encoder {
private:
    int winkel;                    // Aktueller Winkel in Sensorschritten
    int winkelSchritte;            // Anzahl der Schritte pro Umdrehung

    bool running;                  // Läuft der Thread?
    std::thread thread;            // Thread zur Winkelüberwachung

    gpiod_line* a_line;            // Phase A
    gpiod_line* b_line;            // Phase B
	
    struct pollfd fds[2];
    struct gpiod_line_event event;
    EdgeLine eL;
    EdgeLine lastEdge; // für Geschwindigkeit
    int lastWinkel;
    double angleVelocity;
    
    std::queue<EdgeLine> aFlanken;
    std::queue<EdgeLine> bFlanken;
    
    
    
    std::vector<bool> prevABEdge; //vorherige Edge von A und B

    // Thread-Funktion zur Ereignisüberwachung
    void eventLoop();

    // Thread starten
    bool startThread();

    // Thread stoppen
    void stopThread();
    
    void getEvent();
	
    void processEvent();
    void outputEdge(const EdgeLine& e); // zum Debuggen

public:
    // Konstruktor
    Encoder(gpiod_chip* chip, unsigned int pinA, unsigned int pinB,
            int winkelSchritteP = 600);

    // Destruktor
    ~Encoder();

    // Liefert den aktuellen Winkel in Grad zurück
    int getAngle();
    void calculateAngleVelocity();
    double getAngleVelocity();
};

#endif // ENCODER_H

