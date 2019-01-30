/**********************************************************************
 *
 * Filename:    Serial.h
 * 
 * Description: A header file describing the various Serial methods
 *
 * Notes:       
 *
 * 
 * Copyright (c) 2018 by Kelly Klein.  This software is placed into
 * the public domain and may be used for any purpose.  However, this
 * notice must not be changed or removed and no warranty is either
 * expressed or implied by its publication or distribution.
 **********************************************************************/

#ifndef SERIAL_H
#define SERIAL_H

#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include<stdio.h>

#include <string>
#include <vector>
#include <iostream>

#define MAX_SIZE 1024

const int BR2000000	= 2000000;
const int BR1000000	= 1000000;
const int BR230400	= 230400;
const int BR115200	= 115200;
const int BR9600	= 9600;
const int BR19200	= 19200;
const int BR38400	= 38400;
const int BR57600	= 57600;
const int BR1200	= 1200;
const int BR2400	= 2400;
const int BR4800	= 4800;

class Serial{
public:
	/*
	creates new serial object with with either the some of the baudrates I have provided or any int I might have missed.
	You also need to provide the port name. like /dev/ttyAMC0 or /dev/ttyUSB0. If you want the program to sit and wait until
	the next message then set blocking to true. It does require a timeout and the default timeout is 500 milliseconds. As far as I know
	it only accepts integers so set it in units of 100s of milliseconds. And if you need the parity, well...there you go. Its binary. Either 0 parity or 1 parity
	*/
	Serial(int baudRate, std::string port, bool blocking, int timeout = 5, int parity = 0);
	//To absolutly make sure that the port is not left open, the deconstructor closes the port
	~Serial();
	//this just returns one byte from the port
	unsigned char readByte();
	//pretty much just the read function. It reads at most, the len you pass through and returns the number of bytes that were read
	int readData(char *data,int len);
	//pretty much the write function. It writes at most the len you pass it and returns the number of bytes that were read
	int writeData(char *data,int len);
	//same as the read function above but I made it so you can use a vector instead. It will only read at most 1024 bytes
	int readData(std::vector<char> &data);
	//same as the write function above but I made it so you can use a vector instead.
	int writeData(std::vector<char> &data);
	//reads in up to the len you provide or if it reads the newline charactor ('\n'). Returns the number of bytes read
	int readLine(char *data,int len);
	//writes len+1 charactors. It will write the data you provide it followed by the newline charactor ('\n'). Returns the number of bytes read
	int writeLine(char *data,int len);
private:
	int fd;
	std::string port;
	struct termios tty;
};

#endif
