/**********************************************************************
 *
 * Filename:    Serial.cpp
 * 
 * Description: Implementations of Serial class
 *
 * Notes:
 *
 * 
 * Copyright (c) 2018 by Kelly Klein.  This software is placed into
 * the public domain and may be used for any purpose.  However, this
 * notice must not be changed or removed and no warranty is either
 * expressed or implied by its publication or distribution.
 **********************************************************************/

#include "Serial.h"

Serial::Serial(int baudRate, std::string portName, bool blocking, int timeout, int parity){
	port = portName;
	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
	        std::cout << "error " << errno << " opening " << port << ": " << strerror << std::endl;
	        return;
	}

	tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                std::cout << "error " << errno << std::endl;
                return;
        }

        cfsetospeed (&tty, baudRate);
        cfsetispeed (&tty, baudRate);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = blocking ? 1 : 0;
        tty.c_cc[VTIME] = timeout;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                std::cout << "error " << errno << std::endl;
                return;
        }
}

unsigned char Serial::readByte(){
	char buff;
	read(fd,&buff,1);
	return buff;
}

int Serial::readData(char *data,int len){
	int numRead = read(fd,data,len);
	return numRead;
}

int Serial::writeData(char *data,int len){
	int numWrite = write (fd, data, len);
	return numWrite;
}

int Serial::readData(std::vector<char> &data){
	char buff[MAX_SIZE];
	int numRead = read(fd,buff,MAX_SIZE);
	data = std::vector<char>(buff,buff + numRead);

	return numRead;
}

int Serial::writeData(std::vector<char> &data){
	int numWrite = write (fd, data.data(), data.size());
	return numWrite;
}

int Serial::readLine(char *data,int len){
	char next;
	int numRead = 0;
	bool reading = true;
	while(reading && numRead < len){
		read(fd,&next,1);
		if(next == '\n'){
			reading = false;
			break;
		}
		data[numRead] = next;
		numRead++;
	}

	return numRead;
}

int Serial::writeLine(char *data,int len){
	int numWrite = write (fd, data, len);
	write(fd,"\n",1);
	return numWrite;
}

Serial::~Serial(){
	close(fd);
}
