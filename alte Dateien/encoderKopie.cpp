#include "encoder.h"

	std::vector<EdgeMitZeit> Encoder::getEventListe(){
		bool end = false;
		gpiod_line_event eventA;
		gpiod_line_event eventB;
		std::vector<EdgeMitZeit> eventListe;
		//while(!end){
		for(int i = 0; i<4; i++){
			end = true;
			if(eA.eingetragen){
				if(gpiod_line_event_read(a_line, &eventA) == 0){
					end = false;
					if(eA.event.event_type != eventA.event_type){
						eA.event = eventA;
						eA.ts = eventA.ts;
						eA.eingetragen = false;
					}
				}
			}
			if(eB.eingetragen){
				if(gpiod_line_event_read(b_line, &eventB) == 0){
					end = false;
					if(eB.event.event_type != eventB.event_type){
						eB.event = eventB;
						eB.ts = eventB.ts;
						eB.eingetragen = false;
					}
				}
			}
			if(!eA.eingetragen && !eB.eingetragen){
				if(isEarlier(eA.ts, eB.ts)){
					eventListe.push_back(eA);
					eA.eingetragen = true;
				}else{
					eventListe.push_back(eB);
					eB.eingetragen = true;
				}
				continue;
			}
			if(!eA.eingetragen){
				eventListe.push_back(eA);
				eA.eingetragen = true;
				continue;
			}
			if(!eB.eingetragen){
				eventListe.push_back(eB);
				eB.eingetragen = true;
				continue;
			}
		}
		return eventListe;
	}

	bool isEarlier(const timespec &a, const timespec &b) {
		if (a.tv_sec < b.tv_sec)
			return true;
		if (a.tv_sec > b.tv_sec)
			return false;
		// gleiche Sekunden -> nach Nanosekunden vergleichen
		return a.tv_nsec < b.tv_nsec;
	}
	
	int timeApartNS(const timespec &a, const timespec &b){
		long secDiff = b.tv_sec - a.tv_sec;
		long nsecDiff = b.tv_nsec - a.tv_nsec;

		if (nsecDiff < 0) {
			secDiff -= 1;
			nsecDiff += 1000000000L;
		}

		long diff = secDiff * 1000000000L + nsecDiff;

		// Begrenzung auf 2 000 000 000 ns / 2s
		if (diff > 2000000000L)
			diff = 2000000000L;
		else if (diff < -2000000000L)
			diff = -2000000000L;

		return static_cast<int>(diff);	
	}
	

    void Encoder::eventLoop() {
        while(running){
			std::vector<EdgeMitZeit> eventListe = getEventListe();
			//std::cout << "Neue Liste \n";
			for (EdgeMitZeit e : eventListe){
				processEvent(e);
				//std::cout << e.line << " geht auf " 
				//<< e.event.event_type << "  zu  "
				//<< e.event.ts.tv_sec <<"s "
				//<< e.event.ts.tv_nsec << "ns"
				//<<" \n";
			}
			//std::cout << "Ende neue Liste\n";
		}	
    }
    
    
    
    void Encoder::processEvent(const EdgeMitZeit& e) {
		if(e.line == 'B'){
			bHi = e.event.event_type == 2;
			return;
		}
		if(e.event.event_type == 1)return;
		
		if(bHi){
			winkel ++;
		}else{
			winkel --;
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
			int winkelSchritteP){	
		a_line = gpiod_chip_get_line(chip, pinA); // Phase A
        b_line = gpiod_chip_get_line(chip, pinB); // Phase B
        winkel = 0;
        winkelSchritte = winkelSchritteP;
        eA.line = 'A';
        eB.line = 'B';
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
		
