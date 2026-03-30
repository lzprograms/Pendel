#ifndef ENCODER_H
#define ENCODER_H

#include <gpiod.h>
#include <thread>
#include <cmath>
#include <iostream>
#include <vector>
#include <poll.h>
#include <queue>

struct EdgeLine {				// help struct
    char line;			 		// edge 'A' oder 'B'
    gpiod_line_event event;     // original Event
};

bool isEarlier(const timespec &a, const timespec &b);//Hilfsfunktion
int timeApartNS(const timespec &a, const timespec &b);

class Encoder {
private:
    int winkel;                    // current angle in counted flanks

    bool running;                  // is the thread running?
    std::thread thread;            // thread for monitoring angle

    gpiod_line* a_line;            // Phase A
    gpiod_line* b_line;            // Phase B
	
    struct pollfd fds[2];
    struct gpiod_line_event event;
    EdgeLine eL;
    EdgeLine lastEdge; // für Geschwindigkeit
    int lastWinkel;
    int angleVelocity;
    
    std::queue<EdgeLine> aFlanken;
    std::queue<EdgeLine> bFlanken;
    
    
    
    std::vector<bool> prevABEdge; //last edge from a or b

    // thread function for monitoring angle
    void eventLoop();

    // start thread
    bool startThread();
    void stopThread();
    
    void getEvent();
	
    void processEvent();
    void outputEdge(const EdgeLine& e); // for debugging

public:
    // Konstruktor
    Encoder(gpiod_chip* chip, unsigned int pinA, unsigned int pinB,
            int winkelSchritteP = 600);

    // Destruktor
    ~Encoder();

    // give current angle in edge count
    int getAngle();
    void calculateAngleVelocity();
    int getAngleVelocity(); // give angle velocity in edges per second
};

#endif // ENCODER_H

