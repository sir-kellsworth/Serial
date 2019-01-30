#include <Serial.h>

#include <iostream>
#include <vector>

#include <signal.h>

bool running = true;

void signal_handler(int signum){    // signal handler function for key strokes and different events
   if(signum == SIGINT ) { // ctrl+C pressed
      running = false;
   }
}

int main(int argc, char *argv[]){
	Serial serial(BR115200,"/dev/ttyACM0",1);
	signal(SIGINT, signal_handler);

	char data[2];
	std::vector<int> samples(74,0);
	while(running){
		for(int i = 0; i < samples.size(); i++){
			int num = serial.readData(data,2);
			samples[i] = (data[0] << 8) + data[1];
			std::cout << (int)data[0] << " " << (int)data[1] << " " << samples[i] << std::endl;
		}
	}
}
