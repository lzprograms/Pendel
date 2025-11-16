#include "encoder.h"
	
    void Encoder::getEvent(){
	EdgeLine alt;
	alt = eL;
        if (poll(fds, 2, -1) > 0) { // blockiert, bis ein Event kommt
            if (fds[0].revents & POLLIN) {
                gpiod_line_event_read(a_line, &event);
		eL.line = 'A';
		eL.event = event;
		aFlanken.push(eL);
            }
	    if (fds[1].revents & POLLIN) {
                gpiod_line_event_read(b_line, &event);
		eL.line = 'B';
		eL.event = event;
		bFlanken.push(eL);
		
            }
        }else{
		std::cerr << "Fehler: konnte Flanke nicht anfordern\n";
	}
	if(alt.line == eL.line && alt.event.event_type == eL.event.event_type){
		getEvent(); 
		// Falls gleiche Flanke wieder auf gleicher Phase
		// neue Flanke beschaffen
	}
    }
	

void Encoder::eventLoop() {
     while(running){
	if(aFlanken.empty() || bFlanken.empty()){
		getEvent();
	}else{
		processEvent();
	}	
    }
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
    
    
    void Encoder::processEvent() {
	std::vector<bool> newABEdge = prevABEdge;
	//std::cout << "A: " << aFlanken.front().event.event_type <<" B: " << bFlanken.front().event.event_type << "\n";
	if(isEarlier(aFlanken.front().event.ts, bFlanken.front().event.ts)){
		newABEdge[0] = aFlanken.front().event.event_type==1?false:true;
		aFlanken.pop();
	}else{
		newABEdge[1] = bFlanken.front().event.event_type==1?false:true;
		bFlanken.pop();
	}
	int prev =(prevABEdge[0] <<1) | prevABEdge[1];
	int cur = (newABEdge[0] <<1) | newABEdge[1];
	static const int table[4][4] = {
        {  0, +1, -1,  0 }, // von 00
        { -1,  0,  0, +1 }, // von 01
        { +1,  0,  0, -1 }, // von 10
        {  0, -1, +1,  0 }  // von 11
	};	
	winkel +=table[prev][cur];
	calculateAngleVelocity();
	//std::cout << winkel <<"\n";
	//std::cout << prev << cur <<"\n";
	prevABEdge = newABEdge;
    } 
    
    void Encoder::calculateAngleVelocity(){
	int timeApart = timeApartNS(lastEdge.event.ts, eL.event.ts);
	  if(timeApart > 1000000){
		  double aV = static_cast<double>(winkel-lastWinkel)/timeApart;
		  //Winkelschritte pro ns in pro Sekunde ändern
		  aV= aV * 1000000000;
		  angleVelocity = static_cast<int>(aV);
		  lastWinkel = winkel;
		  lastEdge = eL;
		  std::cout << angleVelocity<< "\n";
	  } 
    }
    
    int Encoder::getAngle(){
	    return winkel;
    }
    int Encoder::getAngleVelocity(){
	    return angleVelocity;
    }
    
    void Encoder::outputEdge(const EdgeLine& e) {
	  std::cout << e.line << ":  " << e.event.ts.tv_sec <<"s "
	<< e.event.ts.tv_nsec << "ns" << "\n"; 
	
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
	
	winkel = 0;
	lastWinkel = 0;
        
	fds[0].fd = gpiod_line_event_get_fd(a_line);
	fds[0].events = POLLIN;
	fds[1].fd = gpiod_line_event_get_fd(b_line);
	fds[1].events = POLLIN;
	prevABEdge = {gpiod_line_get_value(a_line)==1?false:true, gpiod_line_get_value(b_line)==1?false:true}; //false=low, true=high, Startzustand festlegen
	
	//gpiod_line_event_read(a_line, &event);
	//eL.event = event;
	eL.line = 'A';
	lastEdge = eL;
	angleVelocity = 0.0;
	
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
		winkelSchritte = winkelSchritteP;
		startThread();
	}
	
	Encoder::~Encoder(){
		stopThread();
		if (a_line) gpiod_line_release(a_line);
		if (b_line) gpiod_line_release(b_line);
	}
		
		
	//double Encoder::getWinkelGrad(){
        //double grad = fmod(winkel * (360.0 / winkelSchritte), 360.0);
        //if (grad < 0) grad += 360.0;
        //return grad;
    //}
		
