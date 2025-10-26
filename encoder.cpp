#include "encoder.h"

    void Encoder::eventLoop() {
        struct gpiod_line_event event;
        while(running){
        if (gpiod_line_event_read(a_line, &event) != 0) continue;
        int B = gpiod_line_get_value(b_line);
        if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
			if (B == 0) {
				winkel++; // A steigt, B ist LOW -> RECHTS
            } else {
                winkel--; // A steigt, B ist HIGH -> LINKS
            }
        }
            
        }
    }
    
    
    bool Encoder::startThread(){
		if(running) return true;
		
		if (!a_line || !b_line) {
             std::cerr << "Fehler: konnte GPIO-Lines nicht abrufen\n";
             return false;
        }
        if (gpiod_line_request_both_edges_events(a_line, "drehgeber") < 0 ||
             gpiod_line_request_both_edges_events(b_line, "drehgeber") < 0){
                std::cerr << "Fehler: konnte Events nicht anfordern\n";
                return false;
        }
        running = true;
        thread = std::thread(&Encoder::eventLoop , this);
        return true; // Thread konnte ohne Fehler gestartet werden
    }
   
     
	void Encoder::stopThread(){
		running = false;
		if (thread.joinable()) {
			thread.join(); // Warten, bis der Thread wirklich beendet ist
		}
	}
	

	Encoder::Encoder(gpiod_chip* chip, unsigned int pinA, unsigned int pinB, 
			int winkelSchritte = 640, double schrittDist = 0.2){	
		chip = chipP;		
		a_line = gpiod_chip_get_line(chip, pinA); // Phase A
        b_line = gpiod_chip_get_line(chip, pinB); // Phase B
        running = true;
		startThread();
	}
	
	Encoder::~Encoder(){
		stopThread();
		if (a_line) gpiod_line_release(a_line);
		if (b_line) gpiod_line_release(b_line);
	}
		
		
	double Encoder::getWinkelGrad(){
        double grad = fmod(winkel * (360.0 / winkelSchritte), 360.0);
        if (grad < 0) grad += 360.0;
        return grad;
    }
		
