#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include "pendel.h"
#include <queue>

bool running;
Pendel* p;
int server_fd, client_fd;

enum class Command {
    calibrateAngle,
    calibratePos,
    calibrateEndPos,
    setPos,
    setRelPos,
    getEndPos,
    getAngle,
    getAngleVelocity,
    getPos,
    setSpeed,
    setMaxSpeed,
    setMaxAcceleration,
    unknown
};
//---
// String wird umgewandelt in ein Enum Command
// somit kann das Kommando mit switch geprüft werden
//---
Command parseCommand(const std::string& cmd) {
    if (cmd.find("setPos")==0) return Command::setPos;
    if (cmd.find("calibratePos")==0) return Command::calibratePos;
    if (cmd.find("setRelPos")==0) return Command::setRelPos;
    if (cmd.find("getAngleVelocity")==0) return Command::getAngleVelocity;
    if (cmd.find("getAngle")==0) return Command::getAngle;
    if (cmd.find("getPos")==0) return Command::getPos;
    if (cmd.find("setSpeed")==0) return Command::setSpeed;
    if (cmd.find("setMaxSpeed")==0) return Command::setMaxSpeed;
    if (cmd.find("setMaxAcceleration")==0) return Command::setMaxAcceleration;
    return Command::unknown;
}

void handleCommand(const std::string& line) {
    int a = 0;
    int b = 0;
    char delimiter = ' ';
    std::queue<std::string> parameters;
    for(int i = 0; i < (int)line.size(); i++){
	if (line[i] == delimiter){
	    b = i;
	    std::string p = line.substr(a, b-a);
	    parameters.push(p);
	    a = b+1;
	}
    }
    if (a < (int)line.size()) {
	parameters.push(line.substr(a));
    }
    std::string command = parameters.front();
    std::string dA; //double Angle (nur falls gebraucht)
    parameters.pop();
    std::string response = "ERR";
    if(p->isCalibrating()){
	response += " CALIBRATING\n";
	send(client_fd, response.c_str(), response.size(), 0);
	return;
    }
    try{
	switch (parseCommand(command)){
	    case Command::setPos:
		if (!parameters.empty()) {
		    int pos = std::stoi(parameters.front());
		    response = p->setPos(pos)?"OK":"ERR limit";
		}
		break;
	    case Command::setRelPos:
		if (!parameters.empty()) {
		    int pos = std::stoi(parameters.front());
		    response = p->setRelPos(pos)?"OK":"ERR limit";
		}
		break;
	    case Command::calibratePos:
		p->calibratePos();
		response = "OK";
		break;
	    case Command::getAngle:
		response = std::to_string(p->getAngle());
		break;
	    case Command::getAngleVelocity:
		//response = std::to_string(p->getAngle());
		//response += " ";
		response = std::to_string(p->getAngleVelocity());
		break;
	    case Command::getPos:
		response = std::to_string(p->getPos());
		break;
	    case Command::setSpeed:
		if (!parameters.empty()) {
		    int speed = std::stoi(parameters.front());
		    response = p->setSpeed(speed)?"OK":"ERR limit";
		}
		break;
	    case Command::setMaxSpeed:
		if (!parameters.empty()) {
		    int speed = std::stoi(parameters.front());
		    response = p->setMaxSpeed(speed)?"OK":"ERR limit";
		}
		break;
	    case Command::setMaxAcceleration:
		if (!parameters.empty()) {
		    int acceleration = std::stoi(parameters.front());
		    response = p->setMaxAcceleration(acceleration)?"OK":"ERR limit";
		}
		break;
	    default:
		response = "ERR unknown";
		std::cout << "Client-Befehl nicht bekannt: " << command <<"\n";
	}
    }catch (const std::invalid_argument& e) {
        response = "ERR parameter";
    } 
    catch (const std::out_of_range& e) {
        response = "ERR parameter";
    }
    response += "\n";
    send(client_fd, response.c_str(), response.size(), 0);
}

int main() {
    p = new Pendel();
    sockaddr_in server_addr{}, client_addr{};
    socklen_t client_len = sizeof(client_addr);

    // Socket erstellen
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("Socket");
        return 1;
    }
    
    int opt = 1;
	setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
	//Adresse freigeben

    // Adresse konfigurieren
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;  // Alle Interfaces
    server_addr.sin_port = htons(8080);        // Port 8080

    // Socket binden
    if (bind(server_fd, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Bind");
        return 1;
    }
    // Warten auf Verbindung
    running = true;
    while(running){
	listen(server_fd, 1);
	std::cout << "Server wartet auf Verbindung...\n";

	client_fd = accept(server_fd, (sockaddr*)&client_addr, &client_len);
	if (client_fd < 0) {
            perror("Accept");
            continue; // try again if accept fails
        }
	std::cout << "Client verbunden!\n";
	std::string message;
	running = true;
	char buffer[1024] = {0};
	message = "";
	bool clientConnected = true;
	while(clientConnected){
	    int bytesRead = read(client_fd, buffer, sizeof(buffer));
	    if (bytesRead <= 0) {
		clientConnected = false;
		close(client_fd);
		break;
	    }
	    message.append(buffer, bytesRead);
	    int newlinePos = message.find('\n');
	    while (newlinePos != -1) {
		std::string line = message.substr(0, newlinePos);
		message.erase(0, newlinePos + 1);
		if (!line.empty()){
		    handleCommand(line);
		    newlinePos = message.find('\n');
		}
	    }
	}
    }

    std::cout << "Client hat Verbindung getrennt! Programm wird beendet!";
    close(client_fd);
    close(server_fd);
    return 0;
}
