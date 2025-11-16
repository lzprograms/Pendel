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
                axis = new Axis(chip, 23, 24, 22, 4, 20, 21, 4000, 0.2);
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
            double Pendel::getPos() const{
                return axis->getPos();
            }
            int Pendel::getAchseLaenge() const{
                return axis->getAchseLaenge();
            }
            bool Pendel::setPos(double pos){
                return axis->setPos(pos);
            }
            bool Pendel::setRelPos(double pos){
                return axis->setPos(axis->getPos()+pos);
            }
            void Pendel::calibratePos(){
                axis->home();
            }
            void Pendel::consoleOut() const{
                std::cout << "\rPos: " << getPos()
                << "   Winkel: " << getAngle()
                << "   Winkelgeschwindigkeit: " << getAngleVelocity()
                << "                   "
                << std::flush;
            }


