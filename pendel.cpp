#include <gpiod.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include "encoder.h"
#include "axis.h"
#include "pendel.h"

//g++ -Wall -std=c++17 -o main main.cpp encoder.cpp axis.cpp -lgpiod -lgpiodcxx -pthread

            Pendel::Pendel(){
                chip = gpiod_chip_open_by_name(chipname);
                if (!chip){
                    throw std::runtime_error("GPIO-Chip nicht gefunden");
                }
                
                encoder = new Encoder(chip, 17, 18);
                axis = new Axis(chip, 23, 24, 22, 4, 20, 21, 5500);
                startPlot();
            }
            
            Pendel::~Pendel() { //GPIO-Pins wieder freigeben
                if (chip)      gpiod_chip_close(chip);
                
            }
            
            int Pendel::getAngle() const{
                return encoder->getAngle();
            }
            int Pendel::getAngleVelocity() const{
                return encoder->getAngleVelocity();
            }
            int Pendel::getPos() const{
                return axis->getPos();
            }
            int Pendel::getEndPos() const{
                return axis->getEndPos();
            }
            bool Pendel::setPos(double pos){
                return axis->setPos(pos);
            }
            bool Pendel::setRelPos(double pos){
                return axis->setPos(axis->getPos()+pos);
            }
            bool Pendel::setSpeed(int speed){
                return axis->setSpeed(speed);
            }
            void Pendel::calibratePos(){
                axis->home();
            }
            bool Pendel::isCalibrating(){
                return axis->isCalibrating();
            }
            bool Pendel::setMaxSpeed(int stepsPerSecond){
                return axis->setMaxSpeed(stepsPerSecond);
            }
            bool Pendel::setMaxAcceleration(int stepsPerSecond2){
                return axis->setMaxAcceleration(stepsPerSecond2);
            }
            void Pendel::consoleOut() const{
                std::cout << "\rPos: " << getPos()
                << "   Winkel: " << getAngle()
                << "   Winkelgeschwindigkeit: " << getAngleVelocity()
                << "                   "
                << std::flush;
            }
            void Pendel::startPlot(){
                if(plotRunning) return;
                thread = std::thread(&Pendel::plotLoop, this);
                plotRunning = true;
            }
            
            void Pendel::plotLoop() const{
                auto next = std::chrono::steady_clock::now(); //start time
                FILE* gp = popen("gnuplot -persistent", "w");
                
                fprintf(gp, "set terminal x11\n");  // oder 'x11'
                std::deque<std::pair<int,int>> buffer;  // ring buffer for values
                int t = 0;
                while (plotRunning) {
                    // send value in real time to gnu
                    int time = t*50;
                    //buffer.push_back({time, encoder->getAngle()});
                    buffer.push_back({time, encoder->getAngle()});
                    if(buffer.size() > 1000)
                        buffer.pop_front();  // delete oldest values
                    
                    if(t% 100 == 99){
                        fprintf(gp, "set xlabel 't [ms]'\n");
                        fprintf(gp, "set ylabel 'Angle [deg]'\n");
                        fprintf(gp, "set yrange [1100:1300]\n");
                        fprintf(gp, "plot '-' with lines title 'Pendulum Angle'\n");
                        for(auto& p : buffer)
                            fprintf(gp, "%d %d\n", p.first, p.second);
                        fprintf(gp, "e\n");
                        fflush(gp);
                    }

                    t++;
                    next += std::chrono::microseconds(50000); // 50 ms
                    std::this_thread::sleep_until(next);
                }

                // end
                fprintf(gp, "e\n");
                fflush(gp);
                pclose(gp);
        }
            


