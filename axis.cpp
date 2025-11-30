#include "axis.h"



      void Axis::eventLoop(){
            auto next = std::chrono::steady_clock::now(); //Startzeit
            int curStep = 1; // changes from 0 to 1 every step
            waitedUs = 0; // waited us since last step 
            minUsDelay = static_cast<int>(1e6 / maxSpeed);
            inverseMaxSpeed = 1.0/maxSpeed;
            curSpeed = 0;
            usDelay = 1e3;
            accumulatedAcceleration = 0;
            int totalAccelerations = 0;
            
                        
            while(running){
                if (usDelay - waitedUs <= minUsDelay){//run atleast every minUsDelay
                    next += std::chrono::microseconds(usDelay - waitedUs);
                    waitedUs = 0;
                }else{
                    next += std::chrono::microseconds(minUsDelay);
                    waitedUs += minUsDelay;
                }
                std::this_thread::sleep_until(next);
                if(tasks.empty())continue; // no tasks? next loop
                MotorTask &curTask = tasks.front();
                //std::cout << minUsDelay << "\n";
                int remainingDistance;
                switch(curTask.state){
                    case MotorState::CHANGEDIRECTION:
                        direction = curTask.arg;
                        gpiod_line_set_value(dir_line, direction==1?1:0);
                        tasks.pop();
                        break;
                    case MotorState::POS:
                        remainingDistance = std::abs(curTask.arg - pos);
                        if(getDecelDistance() < remainingDistance){
                            // if there is still more than enough room to brake, keep accelerating
                            maxAccelerate(maxSpeed);
                        }else{
                            // otherwise initiate brake
                            MotorTask brakeTask;
                            brakeTask.arg = curTask.arg;    // set brake point to desired position
                            brakeTask.state = MotorState::BRAKEPOS;
                            tasks.push(brakeTask);
                            tasks.pop();
                        }
                        break;
                    case MotorState::SPEED:
                        stopOnLimit();
                        if(abs(curSpeed) == abs(curTask.arg))break;
                        maxAccelerate(curTask.arg);
                        totalAccelerations++;
                        break;
                    case MotorState::BRAKEPOS:
                        remainingDistance = std::abs(curTask.arg - pos);
                        if(remainingDistance == 0){
                            tasks.pop();
                            break;
                        }
                        decelerateForPos(remainingDistance);
                        //needed deceleration to stop at desired position, 
                        //formula normally is 2.0 * remainingDistance but pos is only counted every second loop
                        //only on positive steps on gpio pin
                        
                        //std::cout << curSpeed << ": " << curSpeed*curSpeed <<  " / " <<  (2.0 * remainingDistance)  << " * " << ((usDelay -waitedUs)* 1e-6)<< "\n";
                        break;
                    case MotorState::BRAKESPEED:
                        maxDecelerate(curTask.arg);
                        if(curSpeed == curTask.arg)tasks.pop();
                        break;
                    default:
                        break;
                }
                if(curSpeed == 0) continue;
                usDelay = std::max(static_cast<int>(1e6/std::abs(curSpeed)), minUsDelay);
                if(waitedUs != 0) continue; // only step on usDelay, not every loop
                if( // only step on these Motorstates
                    curTask.state == MotorState::POS 
                || curTask.state == MotorState::SPEED 
                || curTask.state ==  MotorState::BRAKEPOS 
                || curTask.state ==  MotorState::BRAKESPEED)
                {
                    curStep = curStep == 1? 0: 1;         // set value of gpio-pin (0/1 -> low/high)
                    pos += direction * curStep; // count total number of steps  
                    gpiod_line_set_value(step_line, curStep);
                }
            }
        }
        
        void Axis::clearQueue(){
            std::queue<MotorTask> empty;    // clear queue
            std::swap( tasks, empty );
        }
        
        int Axis::getDecelDistance(){
            return static_cast<int>((curSpeed * curSpeed) / (2 * maxAcceleration));
        }
        void Axis::accelerate(double value){
            accumulatedAcceleration += value;
            int aa = static_cast<int>(accumulatedAcceleration);
            if(aa != 0){
                curSpeed += aa;
                accumulatedAcceleration -= aa;
            }
        }
        
        void Axis::maxAccelerate(int untilSpeed){
            //std::cout << maxAcceleration << " * " << usDelay-waitedUs << " oder " <<  minUsDelay  << " * " << (maxSpeed - std::abs(curSpeed)) << " * " << direction << "\n";
            //std::cout << (maxAcceleration * std::min(usDelay-waitedUs, minUsDelay) * 1e-6 * (maxSpeed - std::abs(curSpeed)) * inverseMaxSpeed * direction * -1)*60 << "\n";
            if(untilSpeed == curSpeed)return;
            double a = maxAcceleration * std::min(usDelay-waitedUs, minUsDelay) * 1e-6 * (maxSpeed - std::abs(curSpeed)) * inverseMaxSpeed;
            a = std::min(a, static_cast<double>(abs(untilSpeed) - abs(curSpeed)));
            a = a * direction * -1;
            accelerate(a);
            
            
            //std::cout << curSpeed << "\n";
        }
        void Axis::maxDecelerate(int untilSpeed){
            if(untilSpeed == curSpeed)return;
            double a = maxAcceleration * std::min(usDelay-waitedUs, minUsDelay) * 1e-6 * (maxSpeed - std::abs(curSpeed)) * inverseMaxSpeed;
            a = std::min(a, static_cast<double>(abs(curSpeed) - abs(untilSpeed)));
            a = a * direction;
            accelerate(a);
        }
        void Axis::decelerateForPos(int remainingDistance){
            double a = (curSpeed*curSpeed) / (4.0 * remainingDistance) * std::min((usDelay - waitedUs),minUsDelay) * 1e-6;
            a = std::min(a, static_cast<double>(abs(curSpeed)));
            a = a * direction;
            accelerate(a);
        }
        
        void Axis::stopOnLimit(){
            int dd = getDecelDistance();
            if(pos + dd < endPos && direction > 0) return; 
            if(pos - dd > 0      && direction < 0) return;
            clearQueue();
            stopAndReverse(0);
        }
        
        void Axis::stopAndReverse(int direction){
            MotorTask brakeTask;
            brakeTask.arg = 0;    // set brake to speed 0 as first task
            brakeTask.state = MotorState::BRAKESPEED;
            tasks.push(brakeTask);
            MotorTask directionTask;
            directionTask.arg = direction;    // set brake to speed 0 as first task
            directionTask.state = MotorState::CHANGEDIRECTION;
            tasks.push(directionTask);
        }
        
        bool Axis::setPos(int newPos){
            
            if(newPos == pos && curSpeed == 0) return true; 
            // if already stopped at desired position, do nothing
            if(newPos < 0) return false;
            if(newPos > endPos) return false;
            // if desired position out of bounds return false
            clearQueue();
            int distance = newPos - pos;    // distance to new position
            int desiredDirection = distance>0?1:-1;
            
            if(desiredDirection != direction || getDecelDistance() > std::abs(distance))stopAndReverse(desiredDirection);
            MotorTask positionTask;
            positionTask.arg = newPos;    // set desired position as argument
            positionTask.state = MotorState::POS;
            tasks.push(positionTask);
            return true;
        }
        
        bool Axis::setSpeed(int stepsPerSecond){
            if(stepsPerSecond == 0) return false;
            clearQueue();
            int desiredDirection = stepsPerSecond>0?1:-1;
            if(desiredDirection != direction){
                stopAndReverse(desiredDirection);
            }else if(std::abs(curSpeed) > std::abs(stepsPerSecond)){
                MotorTask speedTask;
                speedTask.arg = stepsPerSecond;    // set desired position as argument
                speedTask.state = MotorState::BRAKESPEED;
                tasks.push(speedTask);
            }
            MotorTask speedTask;
            speedTask.arg = stepsPerSecond;    // set desired position as argument
            speedTask.state = MotorState::SPEED;
            tasks.push(speedTask);
            return true;
        }

        
      Axis::Axis(gpiod_chip* chip, unsigned int pinStep,
            unsigned int pinDir, unsigned int pinEn,
            unsigned int pinEndschalter, unsigned int pinMs1,
            unsigned int pinMs2, int endPosP){
               
         endPos = endPosP;
         
         pos = endPos;
         curSpeed = 0;
         direction = -1;
               
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
            gpiod_line_set_value(ms2_line, 1); 
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
            pos = -100; // end limit switch is 100 steps next to usable area
            setPos(static_cast<int>(endPos * 0.5)); // Schlitten in der Mitte positionieren
            return true;
        }
        
        double Axis::getEndPos(){
            return endPos;
        }
        
        int Axis::getPos(){
            return pos;
        }
