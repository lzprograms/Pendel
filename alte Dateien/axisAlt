#include "axis.h"
#include <limits>


      void Axis::eventLoop(){
            auto next = std::chrono::steady_clock::now(); //Startzeit
            int curStep = 1; // changes from 0 to 1 every step
            int waitedUs = 0; // waited us since last step 
            
                        
            while(running){
                if(doingPos){
                    setSpeedForPos();
                }
                if(setUsDelay == std::numeric_limits<int>::max()){
                    //if speed is 0 wait for minimum time and restart loop
                    next += std::chrono::microseconds(minUsDelay);
                    std::this_thread::sleep_until(next);
                    usDelay = minStartUsDelay; //reset speed
                    continue;
                }
                
                
                // setUsDelay is added to minUsDelay -> setUsDelay can be 0
                // minUsDelay is always waited for
                if(setUsDelay > minStartUsDelay){
                    usDelay = setUsDelay;
                }
                if(targetSpeed > 0 && direction != -1){ //if direction changes
                    direction = -1;
                    usDelay = std::max(minStartUsDelay, setUsDelay);
                    //either start with fastest possible speed or a slower set speed
                    gpiod_line_set_value(dir_line, 1);
                } else if(targetSpeed < 0 && direction != 1){
                    direction = 1;
                    usDelay = std::max(minStartUsDelay, setUsDelay);
                    gpiod_line_set_value(dir_line, 0);
                }
                if(direction == -1 && pos >= endPos)waitedUs = -100;
                if(direction == 1 && pos <= 100)   waitedUs = -100;
                // -------
                // if the current delay is greater than the minimum, wait for
                // the minimum delay and increase waitedUs by the waited time.
                // If (delay - waitedTime) is smaller than the minUsDelay,
                // wait only for how much time is left to achieve given delay.
                // This way the loop will repeat  atleast every minUsDelay so that if
                // there is a change in the desired speed, there is no long
                // wait time for a slow step to finish.
                // -------
                //std::cout << waitedUs << " / " << usDelay << "\n";
                if (usDelay - waitedUs <= minUsDelay){
                    next += std::chrono::microseconds(usDelay - waitedUs);
                    std::this_thread::sleep_until(next);
                    gpiod_line_set_value(step_line, curStep); // set value of gpio-pin
                    //std::cout << curStep << "\n";
                    pos += (direction==1?-1:1) * curStep; // count total number of steps
                    curStep = curStep==1?0:1;                 // set value of gpio-pin (0/1 -> low/high)
                    waitedUs = 0;
                    if(setUsDelay < usDelay){
                        usDelay = std::max(usDelay-usDelayDecrease, setUsDelay);
                    }  
                }else{
                    next += std::chrono::microseconds(minUsDelay);
                    waitedUs += minUsDelay;
                    std::this_thread::sleep_until(next);
                }
            }
        }
        
        void Axis::setSpeedForPos(){
            setUsDelay = minUsDelay;
            direction = targetPos > pos?1:0;
            if(targetPos == pos){
                setSpeed(0);
            }
        }
        
      Axis::Axis(gpiod_chip* chip, unsigned int pinStep,
            unsigned int pinDir, unsigned int pinEn,
            unsigned int pinEndschalter, unsigned int pinMs1,
            unsigned int pinMs2, int endPosP){
               
         endPos = endPosP;
         
         pos = endPos;
         targetPos = 0; 
         targetSpeed = 0;
         actualSpeed= 0;   
         direction = 0;
               
         step_line = gpiod_chip_get_line(chip, pinStep);
         dir_line  = gpiod_chip_get_line(chip, pinDir);
         en_line   = gpiod_chip_get_line(chip, pinEn);
         endschalter_line = gpiod_chip_get_line(chip, pinEndschalter);
         
         ms1_line  = gpiod_chip_get_line(chip, pinMs1);
         ms2_line   = gpiod_chip_get_line(chip, pinMs2);
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
            if (!step_line || !dir_line || !en_line || !ms1_line || !ms2_line) {
                std::cerr << "Fehler: konnte Motor-GPIO-Lines nicht abrufen\n";
                return false;
            }
            //GPIO-Pins reservieren
            gpiod_line_request_output(step_line, "step_motor", 0);
            gpiod_line_request_output(dir_line,  "step_motor", 0);
            gpiod_line_request_output(en_line,   "step_motor", 0);
            gpiod_line_request_output(ms1_line,  "step_motor", 0);
            gpiod_line_request_output(ms2_line,   "step_motor", 0);
            
            gpiod_line_set_value(en_line, 0); // Motor aktivieren
            
            gpiod_line_set_value(ms1_line, 1); 
            gpiod_line_set_value(ms2_line, 2); 
            // Microstep auf 8tel Schritte einstellen
            
            
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
            if (ms1_line)  gpiod_line_release(ms1_line);
            if (ms2_line)   gpiod_line_release(ms2_line);
        }
        
        bool Axis::setPos(int newPos){
            if(newPos >= 0 && newPos < endPos){
                targetPos = newPos;
                doingPos = true;
                return true;
            }
            return false;
        }
        bool Axis::setSpeed(int stepsPerSecond){
            doingPos = false;
            if(stepsPerSecond == 0){
                setUsDelay = std::numeric_limits<int>::max();
                return true;
            }else{
                setUsDelay = static_cast<int>(1000000/std::abs(stepsPerSecond));
                setUsDelay = std::max(minUsDelay, setUsDelay);
                //cap setable delay at minimum speed
                direction = stepsPerSecond<0?1:-1;
                gpiod_line_set_value(dir_line, direction==1?1:0);
                return setUsDelay <= minUsDelay;
            }
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
                timeout.tv_sec = 5;    // Timeout wie lange auf Endschaltrer gewartet wird in Sekunden
                
                setSpeed(-3000);
                int ret = gpiod_line_event_wait(endschalter_line, &timeout); 
                if (ret < 0) return false; // Fehler beim Warten auf Flanken (z.B. Line freigegeben)
                if (ret == 0){ // timeout/Zeit abgelaufen -> Endschalter kaputt oder Schlitten steckt fest  
                    setSpeed(0);
                    return false;
                }             
            }    
            pos = -10; // Endschalter liegt 10mm neben der verwendbaren Strecke
            setPos(static_cast<int>(endPos * 0.5)); // Schlitten in der Mitte positionieren
            return true;
        }
        
        double Axis::getEndPos(){
            return endPos;
        }
        
        int Axis::getPos(){
            return pos;
        }
