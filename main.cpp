#include <gpiod.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include "encoder.h"
#include "axis.h"

//g++ -Wall -std=c++17 -o main main.cpp encoder.cpp axis.cpp -lgpiod -lgpiodcxx -pthread


class Pendel {
    private:
        Encoder* encoder;
        Axis* axis;

        const char* chipname = "gpiochip0"; //Fest für Raspberry Pi 3

        gpiod_chip* chip;
    
    public:
        Pendel(){
            chip = gpiod_chip_open_by_name(chipname);
            if (!chip){
                throw std::runtime_error("GPIO-Chip nicht gefunden");
            }
            
            encoder = new Encoder(chip, 6, 5);
            axis = new Axis(chip, 17, 27, 23, 4, 450, 0.2);
        }
        
        ~Pendel() { //GPIO-Pins wieder freigeben
            if (chip)      gpiod_chip_close(chip);
            
        }
        
        double getWinkelGrad(){
            return encoder->getWinkelGrad();
        }
        double getPos(){
            return axis->getPos();
        }
        int getAchseLaenge(){
            return axis->getAchseLaenge();
        }
        bool setPos(double pos){
            return axis->setPos(pos);
        }
    
};



int main() {
    Pendel p;
    const double dt = 0.005; //5ms
    const double Kp = 10.0, Kd = 0, Kx = 0; // Kx sorgt fürs Zentrieren
    const double xMin = 0, 
    xMax = p.getAchseLaenge(), 
    xMitte = p.getAchseLaenge()*0.5;
    const double sollWinkel = 180;
    double letzterWinkel = 0;
    auto next = std::chrono::steady_clock::now(); //Startzeit
    for (int i = 0; i < 50000; ++i) {
        next += std::chrono::microseconds(5000);
        double winkel = p.getWinkelGrad();
        double move = 0;
        if(170 < winkel && winkel < 190){
            double winkelRate = (winkel - letzterWinkel)/dt;
            letzterWinkel = winkel;

            double pos = p.getPos();
            move = Kp * (sollWinkel-winkel) + Kd * winkelRate - Kx * (pos - xMitte);

            pos -= move;
            if (pos < xMin) pos = xMin;
            if (pos > xMax) pos = xMax;
            p.setPos(pos);
        }
        
        if(i%100==0){
            std::cout << "\rPos: " << p.getPos()
            << "   Move: " << move 
            << "   Winkel: " << winkel
            << "    "
            << std::flush;
        }
        std::this_thread::sleep_until(next);
    }
    return 0;
}

