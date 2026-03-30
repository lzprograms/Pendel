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
                        
            while(running){
                std::this_thread::sleep_until(next);
                long timeDeviationUs = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - next
                ).count(); // time between actual step time and wanted step time
                // if jitter occurs due to high cpu load, reset timing
                if (std::abs(timeDeviationUs) > 500) {
                    next = std::chrono::steady_clock::now() + std::chrono::microseconds(minUsDelay);
                }
                
                if (usDelay - waitedUs <= 2 * minUsDelay){//run atleast every minUsDelay
                    next += std::chrono::microseconds(usDelay - waitedUs);
                    waitedUs = 0;
                }else{
                    next += std::chrono::microseconds(minUsDelay);
                    waitedUs += minUsDelay;
                }
                auto maybeTask = tasks.frontIfExists();
                if(!maybeTask)continue; // no tasks? next loop
                MotorTask curTask = *maybeTask;
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
                        break;
                    case MotorState::ACCELERATE:
                    {
                        stopOnLimit();
                        int usSinceLast = usDelay-waitedUs <= 2* minUsDelay ? usDelay-waitedUs : minUsDelay;
                        double maxA = maxAcceleration * usSinceLast * 1e-6 * (maxSpeed - std::abs(curSpeed)) * inverseMaxSpeed;
                        double setA = curTask.arg * usSinceLast * 1e-6;
                        if(setA < -maxA) setA = -maxA;  //cap at maximum allowed Acceleration
                        if(setA > maxA) setA = maxA; 
                        accelerate(setA);
                        if(curSpeed < 0 && direction != 1){
                                direction = 1;
                                gpiod_line_set_value(dir_line, 1);    
                            };
                        if(curSpeed > 0 && direction != -1){
                             direction = -1;
                             gpiod_line_set_value(dir_line, 0);    
                         }
                        break;
                     }   
                    case MotorState::BRAKEPOS:
                        remainingDistance = std::abs(curTask.arg - pos);
                        if(remainingDistance == 0){
                            curSpeed = 0;
                            tasks.pop();
                            break;
                        }
                        decelerateForPos(remainingDistance);
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
                || curTask.state == MotorState::BRAKEPOS 
                || curTask.state == MotorState::BRAKESPEED
                || curTask.state == MotorState::ACCELERATE)
                {
                    curStep = curStep == 1? 0: 1;         // set value of gpio-pin (0/1 -> low/high)
                    pos += direction * curStep; // count total number of steps  
                    checkLimits(); // limit switch triggered?
                    std::this_thread::sleep_until(next);
                    gpiod_line_set_value(step_line, curStep);
                }
            }
        }
        
        void Axis::clearQueue(){
            tasks.clear();
        }
        
        void Axis::checkLimits(){
            int currentRightLimitValue = gpiod_line_get_value(right_limit_line);
            int currentLeftLimitValue = gpiod_line_get_value(left_limit_line);
            if (currentRightLimitValue < 0) { // value below 0 is error
                std::cerr << "Fehler: konnte Endschalter-Line nicht lesen\n";
                return;
            }
            if(currentRightLimitValue != lastRightLimitValue && isHoming) { 
                // once right limit switch is found, go find left limit switch
                setSpeed(2000);
            }
            if (currentRightLimitValue != lastRightLimitValue) { // limit triggered
                pos = 250;
                lastRightLimitValue = currentRightLimitValue;
            }
            if(pos < 250 && currentRightLimitValue != 0){ // if should be at limit but switch is not triggered
                pos = 250; // reset position
            }
            
            
            if(pos > (endPos-250) && currentLeftLimitValue != 0 && !isHoming){ // if should be at limit but switch is not triggered
                pos = (endPos - 250); // reset position
            }
            if(currentLeftLimitValue == 0 && lastLeftLimitValue == 1 && isHoming) { 
                isHoming = false;
                endPos = pos + 250;
                setSpeed(0);
            }
            if(currentLeftLimitValue != lastLeftLimitValue){
                lastLeftLimitValue = currentLeftLimitValue;
            }
        }
        
        int Axis::getDecelDistance(){
            return static_cast<int>((curSpeed * curSpeed) / (2*maxAcceleration));
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
            int usSinceLast = usDelay-waitedUs <= 2* minUsDelay ? usDelay-waitedUs : minUsDelay;
            double a = maxAcceleration * usSinceLast * 1e-6 * (maxSpeed - std::abs(curSpeed)) * inverseMaxSpeed;
            a = std::min(a, static_cast<double>(abs(untilSpeed) - abs(curSpeed)));
            a = a * direction * -1;
            accelerate(a);
            
            
            //std::cout << curSpeed << "\n";
        }
        void Axis::maxDecelerate(int untilSpeed){
            if(untilSpeed == curSpeed)return;
            int usSinceLast = usDelay-waitedUs <= 2* minUsDelay ? usDelay-waitedUs : minUsDelay;
            double a = maxAcceleration * usSinceLast * 1e-6 * (maxSpeed - std::abs(curSpeed)) * inverseMaxSpeed;
            a = std::min(a, static_cast<double>(abs(curSpeed) - abs(untilSpeed)));
            a = a * direction;
            accelerate(a);
        }
        void Axis::decelerateForPos(int remainingDistance){
            int usSinceLast = usDelay-waitedUs <= 2* minUsDelay ? usDelay-waitedUs : minUsDelay;
            double a = (curSpeed*curSpeed) / (4.0 * remainingDistance) * usSinceLast * 1e-6;
            a = std::min(a, static_cast<double>(abs(curSpeed)));
            a = a * direction;
            accelerate(a);
        }
        
        void Axis::stopOnLimit(){ // stop in time
            //if(isHoming)return; // ignore previous limits if homing
            int dd = getDecelDistance();
            if(curSpeed == 0) return; 
            if((pos + dd > endPos && curSpeed < 0 )||( pos - dd < 0      && curSpeed > 0)){
                clearQueue();
                stopAndReverse(direction);
                isHoming = false; // homing completed
            }
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
            //if(stepsPerSecond == 0) return false;
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
        bool Axis::setAcceleration(int stepsPerSecond){
            clearQueue();
            if(curSpeed == 0){
                if(stepsPerSecond > 0){
                    direction = -1;
                    gpiod_line_set_value(dir_line, 0);    
                }else{
                    direction = 1;
                    gpiod_line_set_value(dir_line, 1);  
                }
            }
            MotorTask accelerateTask;
            accelerateTask.arg = stepsPerSecond;    // set desired position as argument
            accelerateTask.state = MotorState::ACCELERATE;
            tasks.push(accelerateTask);
            return true;
        }

        
      Axis::Axis(gpiod_chip* chip, unsigned int pinStep,
            unsigned int pinDir, unsigned int pinEn,
            unsigned int pinRightLimit, unsigned int pinLeftLimit,
            unsigned int pinMs1, unsigned int pinMs2){
               
         endPos = 10000;
         
         pos = endPos;
         curSpeed = 0;
         direction = -1;
         isHoming = false;
               
         step_line = gpiod_chip_get_line(chip, pinStep);
         dir_line  = gpiod_chip_get_line(chip, pinDir);
         en_line   = gpiod_chip_get_line(chip, pinEn);
         right_limit_line = gpiod_chip_get_line(chip, pinRightLimit);
         left_limit_line = gpiod_chip_get_line(chip, pinLeftLimit);
         
         ms1_line  = gpiod_chip_get_line(chip, pinMs1);
         ms2_line   = gpiod_chip_get_line(chip, pinMs2);
         
         if (gpiod_line_request_both_edges_events(right_limit_line, "home") < 0) 
        {
            std::cerr << "Fehler: konnte Events nicht anfordern(Home)\n";
        }
        if (gpiod_line_request_both_edges_events(left_limit_line, "home") < 0) 
        {
            std::cerr << "Fehler: konnte Events nicht anfordern(Home)\n";
        }
         
         home();
         startThread();
      }
      
      Axis::~Axis() {
        stopThread();
        if (right_limit_line)   gpiod_line_release(right_limit_line);
        if (left_limit_line)   gpiod_line_release(left_limit_line);
      }
      
      
      bool Axis::startThread(){
            if(running) return true;
            if (!step_line || !dir_line || !en_line || !ms1_line || !ms2_line) {
                std::cerr << "Fehler: konnte Motor-GPIO-Lines nicht abrufen\n";
                return false;
            }
            //reserve GPIO-Pins
            gpiod_line_request_output(step_line, "step_motor", 0);
            gpiod_line_request_output(dir_line,  "step_motor", 0);
            gpiod_line_request_output(en_line,   "step_motor", 0);
            gpiod_line_request_output(ms1_line,  "step_motor", 0);
            gpiod_line_request_output(ms2_line,   "step_motor", 0);
            
            gpiod_line_set_value(en_line, 0); // turn motor on, 0 = ON, 1 = OFF
            
            gpiod_line_set_value(ms1_line, 1); 
            gpiod_line_set_value(ms2_line, 1); 
            // Set microstep mode to 1/8
            
            running = true; // thread is started
            thread = std::thread(&Axis::eventLoop, this);
            return true; //thread was started with no error
        }
        
        void Axis::stopThread(){
            gpiod_line_set_value(en_line, 1); // turn motor off
            running = false;
            if (thread.joinable()) thread.join();
            if (step_line) gpiod_line_release(step_line);
            if (dir_line)  gpiod_line_release(dir_line);
            if (en_line)   gpiod_line_release(en_line);
            if (ms1_line)  gpiod_line_release(ms1_line);
            if (ms2_line)   gpiod_line_release(ms2_line);
        }
        
        
        bool Axis::home(){ // Schlittenposition ermitteln
            isHoming = true;
            int currentRightLimitValue = gpiod_line_get_value(right_limit_line);
            int currentLeftLimitValue = gpiod_line_get_value(left_limit_line);
            if (currentRightLimitValue < 0) { // value below 0 is error
                std::cerr << "Fehler: konnte Endschalter-Line nicht lesen\n";
                return false;
            }
            
            if(currentRightLimitValue == 0){ // already at limit?
                pos = -500;
                setSpeed(2000); // untrigger limit switch by going forward
            }else{
                pos = endPos*1.2;
                setSpeed(-2000); // trigger limit switch by going backwards
            }
            lastRightLimitValue = currentRightLimitValue;
            lastLeftLimitValue = currentLeftLimitValue;
            return true;
        }
        
        bool Axis::isCalibrating(){
            return isHoming;
        }
        bool Axis::setMaxSpeed(int stepsPerSecond){
            if(stepsPerSecond < 1) return false;
            maxSpeed = stepsPerSecond;
            return true;
        }
        bool Axis::setMaxAcceleration(int stepsPerSecond2){
            if(stepsPerSecond2 < 1) return false;
            maxAcceleration = stepsPerSecond2;
            return true;
        }
        
        double Axis::getEndPos(){
            return endPos;
        }
        
        int Axis::getPos(){
            return pos;
        }
        int Axis::getSpeed(){
            return curSpeed;
        }
