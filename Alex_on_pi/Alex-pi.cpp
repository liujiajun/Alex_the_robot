#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include <string>
#include <sstream>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include "constants.h"
using namespace std;
#define PORT_NAME			"/dev/ttyACM0"
#define BAUD_RATE			B57600

int exitFlag=0;
sem_t _xmitSema;
//gcc Alex-pi.cpp serial.cpp serialize.cpp -pthread -o Alex-pi
void handleError(TResult error)
{
	switch(error)
	{
		case PACKET_BAD:
			cout<<"ERROR: Bad Magic Number\n";
			break;

		case PACKET_CHECKSUM_BAD:
			cout<<"ERROR: Bad checksum\n";
			break;

		default:
			cout<<"ERROR: UNKNOWN ERROR\n";
	}
}

void handleStatus(TPacket *packet)
{
	printf("\n ------- ALEX STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", packet->params[0]);
	printf("Right Forward Ticks:\t\t%d\n", packet->params[1]);
	printf("Left Reverse Ticks:\t\t%d\n", packet->params[2]);
	printf("Right Reverse Ticks:\t\t%d\n", packet->params[3]);
	printf("Left Forward Ticks Turns:\t%d\n", packet->params[4]);
	printf("Right Forward Ticks Turns:\t%d\n", packet->params[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", packet->params[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", packet->params[7]);
	printf("Forward Distance:\t\t%d\n", packet->params[8]);
	printf("Reverse Distance:\t\t%d\n", packet->params[9]);
	printf("\n---------------------------------------\n\n");
}
void handleColor(TPacket *packet)
{
	printf("\nRed: %d\n", packet -> params[0]);
	printf("Green: %d\n", packet -> params[1]);
	printf("Blue: %d\n", packet -> params[2]);
	char color; 
	if (packet -> params[1] - packet -> params[0] > 30) color = 'R';
	else color = 'G';
	printf("Color: %c\n", color);
}
void handleIR(TPacket *packet)
{
	printf("\nIR1: %d\n", packet -> params[0]);
	printf("IR2: %d\n", packet -> params[1]);
	printf("IR3: %d\n", packet -> params[2]);
}
void handleResponse(TPacket *packet)
{
	// The response code is stored in command
	switch(packet->command)
	{
		case RESP_OK:
			cout<<"Command OK\n";
		break;

		case RESP_STATUS:
			handleStatus(packet);
		break;
		
		case RESP_COLOR:
			handleColor(packet);
		break;
		case RESP_IR:
			handleIR(packet);
		break;
		default:
			cout<<"Alex is confused.\n";
	}
}

void handleErrorResponse(TPacket *packet)
{
	// The error code is returned in command
	switch(packet->command)
	{
		case RESP_BAD_PACKET:
			cout<<"Arduino received bad magic number\n";
		break;

		case RESP_BAD_CHECKSUM:
			cout<<"Arduino received bad checksum\n";
		break;

		case RESP_BAD_COMMAND:
			cout<<"Arduino received bad command\n";
		break;

		case RESP_BAD_RESPONSE:
			cout<<"Arduino received unexpected response\n";
		break;

		default:
			cout<<"Arduino reports a weird error\n";
	}
}

void handleMessage(TPacket *packet)
{
	printf("Message from Alex: %s\n", packet->data);
}

void handlePacket(TPacket *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
				// Only we send command packets, so ignore
			break;

		case PACKET_TYPE_RESPONSE:
				handleResponse(packet);
			break;

		case PACKET_TYPE_ERROR:
				handleErrorResponse(packet);
			break;

		case PACKET_TYPE_MESSAGE:
				handleMessage(packet);
			break;
	}
}

void sendPacket(TPacket *packet)
{
	char buffer[PACKET_SIZE];
	int len = serialize(buffer, packet, sizeof(TPacket));

	serialWrite(buffer, len);
}

void *receiveThread(void *p)
{
	char buffer[PACKET_SIZE];
	int len;
	TPacket packet;
	TResult result;
	int counter=0;

	while(1)
	{
		len = serialRead(buffer);
		counter+=len;
		if(len > 0)
		{
			result = deserialize(buffer, len, &packet);

			if(result == PACKET_OK)
			{
				counter=0;
				handlePacket(&packet);
			}
			else 
				if(result != PACKET_INCOMPLETE)
				{
					cout<<"PACKET ERROR\n";
					handleError(result);
				}
		}
	}
}

void flushInput()
{
	char c;

	while((c = getchar()) != '\n' && c != EOF);
}

void getParams(TPacket *commandPacket)
{
	printf("Enter distance/angle and power: ");
	scanf("%d %d", &commandPacket->params[0], &commandPacket->params[1]);
	flushInput();
}

void sendCommand(char command, int a, int b)
{
	TPacket commandPacket;

	commandPacket.packetType = PACKET_TYPE_COMMAND;
	commandPacket.params[0] = a;
	commandPacket.params[1] = b;
	switch(command)
	{
		case 'f':
		case 'F':
			//getParams(&commandPacket);
			commandPacket.command = COMMAND_FORWARD;
			sendPacket(&commandPacket);
			break;

		case 'b':
		case 'B':
			//getParams(&commandPacket);
			commandPacket.command = COMMAND_REVERSE;
			sendPacket(&commandPacket);
			break;

		case 'l':
		case 'L':
			//getParams(&commandPacket);
			commandPacket.command = COMMAND_TURN_LEFT;
			sendPacket(&commandPacket);
			break;

		case 'r':
		case 'R':
			//getParams(&commandPacket);
			commandPacket.command = COMMAND_TURN_RIGHT;
			sendPacket(&commandPacket);
			break;

		case 's':
		case 'S':
			commandPacket.command = COMMAND_STOP;
			sendPacket(&commandPacket);
			break;

		case 'c':
		case 'C':
			commandPacket.command = COMMAND_CLEAR_STATS;
			commandPacket.params[0] = 0;
			sendPacket(&commandPacket);
			break;

		case 'g':
		case 'G':
			commandPacket.command = COMMAND_GET_STATS;
			sendPacket(&commandPacket);
			break;

		case 'y':
		case 'Y':
			commandPacket.command = COMMAND_GET_COLOR;
			sendPacket(&commandPacket);
			break;
		case 'i':
		case 'I':
			commandPacket.command = COMMAND_GET_IR;
			sendPacket(&commandPacket);
			break;
		case 'q':
		case 'Q':
			exitFlag=1;
			break;

			
		default:
			printf("Bad command\n");

	}
}

int main()
{
	// Connect to the Arduino
	startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);

	// Sleep for two seconds
	printf("WAITING TWO SECONDS FOR ARDUINO TO REBOOT\n");
	sleep(2);
	printf("DONE\n");

	// Spawn receiver thread
	pthread_t recv;

	pthread_create(&recv, NULL, receiveThread, NULL);

	// Send a hello packet
	TPacket helloPacket;

	helloPacket.packetType = PACKET_TYPE_HELLO;
	sendPacket(&helloPacket);

	while(!exitFlag)
	{
		//char ch;
		//cout<<"Command: ";
		//scanf("%c", &ch);
		string s; getline(cin, s);
		int a, b;
		if (s[0] == 'f' || s[0] == 'b'){
			a = 10; b = 130;
		} else if (s[0] == 'l' || s[0] == 'r'){
			a = 15; b = 200;
		}
		if (s.size() > 1){
			stringstream ss(s);
			char dummy; ss>>dummy>>a>>b;
		}
		// Purge extraneous characters from input stream
		//flushInput(); cin.clear();
		sendCommand(s[0], a, b);
	}

	printf("Closing connection to Arduino.\n");
	endSerial();
}
