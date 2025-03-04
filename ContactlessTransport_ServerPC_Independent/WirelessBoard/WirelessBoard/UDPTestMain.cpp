// This main file is for simply testing the UDP communication
#include <src\UDPDriver.h>
#include <src\COMToolkit2.h>
#include <conio.h>

void main(void) {
//	UDPDriver udpDriver;
//	char serverIP[] = "192.168.0.140";
//	int socketPort = 54007;
//	char clientIP[] = "192.168.0.207";
//	udpDriver.initialise(serverIP, 54007);
//	unsigned int client = udpDriver.addClient(clientIP);
//	bool stop = false;
//	int count = 1;
//	unsigned char message[2] = { 0,0 };
//	while (!stop) {
//		if (_kbhit()) {
//			switch (_getch()) {
//			case 'X':
//			case 'x':
//				stop = true;
//			}
//		}
//		message[0] = message[1];
//		message[1] = count;
//		udpDriver.sendString(message, 1);
//		printf("Sent %d&%d\n", (int)message[0], (int)message[1]);
//		count++;
//		Sleep(500);
//	}
//	udpDriver.disconnect();


	COMToolkit2 comDriver;
	comDriver.connect(13, 2000000);
	bool stop = false;
	char count = 0;
	while (!stop) {
		if (_kbhit()) {
			switch (_getch()) {
			case 'X':
			case 'x':
				stop = true;
			}
		}
		comDriver.sendByte(count);
		printf("Sent %d\n", (int)count);
		count++;
	}
	comDriver.closeConnection();
}

