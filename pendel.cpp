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


