#include "axis.h"


      Axis::eventLoop(){
            auto next = std::chrono::steady_clock::now(); //Startzeit
            while(running){
                double dif = pos-sollPos; // Differenz zw. Soll- und Ist-Position
                next += std::chrono::microseconds(usDelay1);
                if(abs(dif) >= schrittDist){ //Differenz mehr als halbe Schrittlänge?
                        gpiod_line_set_value(dir_line, dif>0?0:1);
                        gpiod_line_set_value(step_line, 1);
                        std::this_thread::sleep_until(next);
                        next += std::chrono::microseconds(usDelay2);
                        gpiod_line_set_value(step_line, 0);
                        pos += (dif > 0 ? -schrittDist : schrittDist);
                }
                std::this_thread::sleep_until(next);
                
            }
        }
        
      Axis::Axis(gpiod_chip* chip, unsigned int pinStep,
            unsigned int pinDir, unsigned int pinEn,
            unsigned int pinEndschalter, int achseLaengeP,
            double schrittDistP){
               
         achseLaenge = achseLaengeP;
         schrittDist = schrittDistP;  
         
         pos = 0;
         sollPos = 0;    
               
         step_line = gpiod_chip_get_line(chip, pinStep);
         dir_line  = gpiod_chip_get_line(chip, pinDir);
         en_line   = gpiod_chip_get_line(chip, pinEn);
         endschalter_line = gpiod_chip_get_line(chip, pinEndschalter);
         
         startThread();
         if (!home()) {
            throw std::runtime_error("Konnte Schlitten nicht zurücksetzen");
         }
      }
      
      Axis::~Axis() {
        stopThread();
        if (endschalter_line)   gpiod_line_release(endschalter_line);
      }
      
      
      bool Axis::startThread(){
            if(running) return true;
         
            if (!step_line || !dir_line || !en_line) {
                std::cerr << "Fehler: konnte Motor-GPIO-Lines nicht abrufen\n";
                return false;
            }
            //GPIO-Pins reservieren
            gpiod_line_request_output(step_line, "step_motor", 0);
            gpiod_line_request_output(dir_line,  "step_motor", 0);
            gpiod_line_request_output(en_line,   "step_motor", 0);
            
            gpiod_line_set_value(en_line, 0); // Motor aktivieren
            
            running = true;
            thread = std::thread(&Axis::eventLoop, this);
            return true; //Thread konnte ohne Fehler gestarten werden
        }
        
        void Axis::stopThread(){
            gpiod_line_set_value(en_line, 1); // Motor deaktivieren
            running = false;
            if (thread.joinable()) thread.join();
            if (step_line) gpiod_line_release(step_line);
            if (dir_line)  gpiod_line_release(dir_line);
            if (en_line)   gpiod_line_release(en_line);
        }
        
        bool Axis::setPos(double newPos){
            if(newPos >= 0 && newPos < achseLaenge){
                sollPos = newPos;
                return true;
            }
            return false;
        }
        
        bool Axis::home(){ // Schlittenposition ermitteln
            if (gpiod_line_request_both_edges_events(endschalter_line, "home") < 0) 
                {
                std::cerr << "Fehler: konnte Events nicht anfordern\n";
                return false;
            }
              // Vorab prüfen: Schlitten schon am Endschalter?
            int val = gpiod_line_get_value(endschalter_line);
            if (val < 0) {
                std::cerr << "Fehler: konnte Line nicht lesen\n";
                return false;
            }
            
            if (val != 0) { // Endschalter nicht schon gedrückt (LOW = aktiv)
                struct timespec timeout;
                timeout.tv_sec = 4;    // Timeout wie lange auf Endschaltrer gewartet wird in Sekunden
                
                sollPos = -achseLaenge; //So lange fahren bis Endschalter gedrückt
                int ret = gpiod_line_event_wait(endschalter_line, &timeout); 
                if (ret < 0) return false; // Fehler beim Warten auf Flanken (z.B. Line freigegeben)
                if (ret == 0){ // timeout/Zeit abgelaufen -> Endschalter kaputt oder Schlitten steckt fest  
                    sollPos = pos; //Schlitten stoppen
                    return false;
                }             
            }    
            pos = -10; // Endschalter liegt 10mm neben der verwendbaren Strecke
            sollPos = achseLaenge * 0.5; // Schlitten in der Mitte positionieren
            return true;
        }
        
        double Axis::getAchseLaenge(){
            return achseLaenge;
        }
        
        double Axis::getPos(){
            return pos;
        }
