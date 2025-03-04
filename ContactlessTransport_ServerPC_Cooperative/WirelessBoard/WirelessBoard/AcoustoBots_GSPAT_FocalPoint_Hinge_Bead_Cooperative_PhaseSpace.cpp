#include <src/UDPDriver.h>
#include <AsierInhoUDP.h>
#include <GSPAT_SolverV2.h>
#include <Helper\HelperMethods.h>
#include <Helper\microTimer.h>
#include <string.h>
#include <conio.h>

UDPDriver udp;

int packetSize = 128;

//Setting the board
const int numPatBoards = 2; // number of PAT boards
const int numMonaRobots = 2; // number of MONA robots
int PATs[numPatBoards], Monas[numMonaRobots];

// PhaseSpaceTracking
#include <iostream>
#include <conio.h>
#include <WinSock2.h>
#include <ctime>
#include <string>
#include <map>
#include <Windows.h>
#include <process.h>
#include <conio.h>

#include <WS2tcpip.h>
#include <vector>
#include <string.h>

#pragma comment (lib, "ws2_32.lib")
using namespace std;

//UDP port we will listen to:
#define PORT 7777
SOCKET sockPhaseSpace;
struct sockaddr_in recv_addr;

int Bot1, Bot2, Bot3, Bot4, Bot5;
float Bot1_X, Bot1_Y, Bot1_Z, Bot2_X, Bot2_Y, Bot2_Z, Bot3_X, Bot3_Y, Bot3_Z, Bot4_X, Bot4_Y, Bot4_Z, Bot5_X, Bot5_Y, Bot5_Z;
float Bot1_Orientation, Bot2_Orientation, Bot3_Orientation, Bot4_Orientation, Bot5_Orientation;

float Bot1_X_d, Bot1_Y_d, Bot1_Z_d, Bot2_X_d, Bot2_Y_d, Bot2_Z_d, Bot3_X_d, Bot3_Y_d, Bot3_Z_d, Bot4_X_d, Bot4_Y_d, Bot4_Z_d, Bot5_X_d, Bot5_Y_d, Bot5_Z_d;
float Bot1_Orientation_d, Bot2_Orientation_d, Bot3_Orientation_d, Bot4_Orientation_d, Bot5_Orientation_d;

int User1Left, User1Right, User2Left, User2Right;
float User1Left_X, User1Left_Y, User1Left_Z, User1Right_X, User1Right_Y, User1Right_Z, User2Left_X, User2Left_Y, User2Left_Z, User2Right_X, User2Right_Y, User2Right_Z;
float User1Left_Orientation, User1Right_Orientation, User2Left_Orientation, User2Right_Orientation;

//Data Structure containing tracking info:
struct TrackerState {
	float position[3];
	float pose[4];
	float headingY;
	struct _OWL_Marker {
		unsigned int ID;
		float x, y, z;
		float timestamp;
	};
	_OWL_Marker markers[4];
};

//Data Structure containing tracking info:
struct BotPose {
	float position[3];
	float orientation;
};

/**
Buffer where we will receive the data:
It has space for 12 trackers:     numTrackers + 12*( trackerName    +    TrackerState    ) 
*/
static const int MAX_SIZE_IN_BYTES = sizeof(int) + 12 * (32 * sizeof(char) + sizeof(TrackerState));
char recvBuff[MAX_SIZE_IN_BYTES];

//0. Map where we will store most up-to-date tracking information (last update from each tracker)
// map<string, struct TrackerState> trackingDatabase;

//3. Print current tracking database:
// std::map<string, TrackerState>::iterator itDatabase;

/**
Receive UDP messages encoded by the tracking system.
Messages will be encoded in bynary format, and a message will contain:
	1. int numTrackers: Number of tracker informations encoded in this update.
	2. char trackerName[32]: The name of the tracker encoded ALWAYS with 32 chars.
	3. TrackerState: The structure with all data (the client needs to load data into the same structure. DEFINITIONS NUST MATCH ON BOTH SIDES).

	For example, an update with 2 trackers will look like
	_________________________________________________________________________________________________________________________
	| numTrackers (1 int) | trackerName1 (32 char) | TrackerState1 (struct)| trackerName2 (32 char) | TrackerState2 (struct)|
	_________________________________________________________________________________________________________________________

	Please note parts 2&3 will repeat once per tracker (for a total of "numTracker" repetitions).
*/

void print(const char* str) {
	printf("%s\n", str);
}

void updatePhasesAndAmplitudes(AsierInhoUDP::AsierInhoUDPBoard* driver, float* phases, float* amplitudes) {
	unsigned char* message = driver->discretize(phases, amplitudes);
	driver->updateMessage(message);
}

void recvFunct(void* param)
{
	//0. Map where we will store most up-to-date tracking information (last update from each tracker)
	map<string, struct TrackerState> trackingDatabase;

	// printf("Receiving data: PRESS KEY TO STOP:");
	// while (!_kbhit())
	// {
		//1. Receive package with data from ALL trackers.
		sockaddr msgSender;
		int msgSize = sizeof(msgSender);
		int bytesReceived = recvfrom(sockPhaseSpace, recvBuff, MAX_SIZE_IN_BYTES, 0, (sockaddr*)&msgSender, &msgSize);

		//2. Parse the packet (parsingIndex points to the part of the package we are parsing): 
		char* parsingIndex = recvBuff;
		//2.1. Parse numTrackers (1 int):  
		int numTrackers = *((int*)parsingIndex);  //Parse an int (numTrackers is encoded as int)
		parsingIndex += sizeof(int);                //Move our index by the size of an int (we read it already), to read remaining info. 
		//2.2. Parse ALL the tracking info (repeat numTracker times): 
		for (int t = 0; t < numTrackers; t++) {
			//a) Read the name of the tracker (32 chars):
			char trackerName[32];
			memcpy(trackerName, parsingIndex, 32 * sizeof(char)); //Copy 32 chars (one name is ALWAYS 32 chars).
			parsingIndex += 32 * sizeof(char);                    //Move index by the size of a trackerName (32 chars), to keep reading.
			//b) Read the TrackerData: 
			TrackerState curState;
			memcpy(&curState, parsingIndex, sizeof(TrackerState));//Copy one TrackerState (one struct TrackerState)
			parsingIndex += sizeof(TrackerState);
			//c) store in the database: 
			trackingDatabase[string(trackerName)] = curState;
		}

		//3. Print current tracking database:
		std::map<string, TrackerState>::iterator itDatabase;
		for (itDatabase = trackingDatabase.begin(); itDatabase != trackingDatabase.end(); itDatabase++) {
			/* printf("ID: %s:\n", itDatabase->first.c_str());
			printf("\tPosition: %f, %f, %f\n", itDatabase->second.position[0], itDatabase->second.position[1], itDatabase->second.position[2]);
			printf("\tPose: %f, %f, %f, %f\n", itDatabase->second.pose[0], itDatabase->second.pose[1], itDatabase->second.pose[2], itDatabase->second.pose[3]);
			printf("\tHeading(Y): %f\n", itDatabase->second.headingY); */
			
			std::string AcoustoBot1 = "AcoustoBot1";
			if (strcmp(itDatabase->first.c_str(), AcoustoBot1.c_str()) == 0) {
				// printf("ID: %s:\n", itDatabase->first.c_str());
				// printf("\tPosition: %f, %f, %f\n", itDatabase->second.position[0], itDatabase->second.position[1], itDatabase->second.position[2]);
                Bot1 = 1;
				Bot1_X = itDatabase->second.position[0];
				Bot1_Y = itDatabase->second.position[1];
				Bot1_Z = itDatabase->second.position[2];
				// printf("\tPose: %f, %f, %f, %f\n", itDatabase->second.pose[0], itDatabase->second.pose[1], itDatabase->second.pose[2], itDatabase->second.pose[3]);
				// printf("\tHeading(Y): %f\n", itDatabase->second.headingY);
				Bot1_Orientation = itDatabase->second.headingY;
			} 
			
			std::string AcoustoBot2 = "AcoustoBot2";
			if (strcmp(itDatabase->first.c_str(), AcoustoBot2.c_str()) == 0) {
				// printf("ID: %s:\n", itDatabase->first.c_str());
				// printf("\tPosition: %f, %f, %f\n", itDatabase->second.position[0], itDatabase->second.position[1], itDatabase->second.position[2]);
                Bot2 = 2;
				Bot2_X = itDatabase->second.position[0];
				Bot2_Y = itDatabase->second.position[1];
				Bot2_Z = itDatabase->second.position[2];
				// printf("\tPose: %f, %f, %f, %f\n", itDatabase->second.pose[0], itDatabase->second.pose[1], itDatabase->second.pose[2], itDatabase->second.pose[3]);
				// printf("\tHeading(Y): %f\n", itDatabase->second.headingY);
				Bot2_Orientation = itDatabase->second.headingY;
			}
			
			std::string AcoustoBot3 = "AcoustoBot3";
			if (strcmp(itDatabase->first.c_str(), AcoustoBot3.c_str()) == 0) {
				// printf("ID: %s:\n", itDatabase->first.c_str());
				// printf("\tPosition: %f, %f, %f\n", itDatabase->second.position[0], itDatabase->second.position[1], itDatabase->second.position[2]);
                Bot3 = 3;
				Bot3_X = itDatabase->second.position[0];
				Bot3_Y = itDatabase->second.position[1];
				Bot3_Z = itDatabase->second.position[2];
				// printf("\tPose: %f, %f, %f, %f\n", itDatabase->second.pose[0], itDatabase->second.pose[1], itDatabase->second.pose[2], itDatabase->second.pose[3]);
				// printf("\tHeading(Y): %f\n", itDatabase->second.headingY);
				Bot3_Orientation = itDatabase->second.headingY;
			}
			
			std::string AcoustoBot4 = "AcoustoBot4";
			if (strcmp(itDatabase->first.c_str(), AcoustoBot4.c_str()) == 0) {
				// printf("ID: %s:\n", itDatabase->first.c_str());
				// printf("\tPosition: %f, %f, %f\n", itDatabase->second.position[0], itDatabase->second.position[1], itDatabase->second.position[2]);
				Bot4 = 4;
				Bot4_X = itDatabase->second.position[0];
				Bot4_Y = itDatabase->second.position[1];
				Bot4_Z = itDatabase->second.position[2];
				// printf("\tPose: %f, %f, %f, %f\n", itDatabase->second.pose[0], itDatabase->second.pose[1], itDatabase->second.pose[2], itDatabase->second.pose[3]);
				// printf("\tHeading(Y): %f\n", itDatabase->second.headingY);
				Bot4_Orientation = itDatabase->second.headingY;
			}
			
			std::string AcoustoBot5 = "AcoustoBot5";
			if (strcmp(itDatabase->first.c_str(), AcoustoBot5.c_str()) == 0) {
				// printf("ID: %s:\n", itDatabase->first.c_str());
				// printf("\tPosition: %f, %f, %f\n", itDatabase->second.position[0], itDatabase->second.position[1], itDatabase->second.position[2]);
				Bot5 = 5;
				Bot5_X = itDatabase->second.position[0];
				Bot5_Y = itDatabase->second.position[1];
				Bot5_Z = itDatabase->second.position[2];
				// printf("\tPose: %f, %f, %f, %f\n", itDatabase->second.pose[0], itDatabase->second.pose[1], itDatabase->second.pose[2], itDatabase->second.pose[3]);
				// printf("\tHeading(Y): %f\n", itDatabase->second.headingY);
				Bot5_Orientation = itDatabase->second.headingY;
			}
			
			std::string User1Left = "User1Left";
			if (strcmp(itDatabase->first.c_str(), User1Left.c_str()) == 0) {
				// printf("ID: %s:\n", itDatabase->first.c_str());
				// printf("\tPosition: %f, %f, %f\n", itDatabase->second.position[0], itDatabase->second.position[1], itDatabase->second.position[2]);
				User1Left = 6;
				User1Left_X = itDatabase->second.position[0];
				User1Left_Y = itDatabase->second.position[1];
				User1Left_Z = itDatabase->second.position[2];
				// printf("\tPose: %f, %f, %f, %f\n", itDatabase->second.pose[0], itDatabase->second.pose[1], itDatabase->second.pose[2], itDatabase->second.pose[3]);
				// printf("\tHeading(Y): %f\n", itDatabase->second.headingY);
				User1Left_Orientation = itDatabase->second.headingY;
			}
			
			std::string User1Right = "User1Right";
			if (strcmp(itDatabase->first.c_str(), User1Right.c_str()) == 0) {
				// printf("ID: %s:\n", itDatabase->first.c_str());
				// printf("\tPosition: %f, %f, %f\n", itDatabase->second.position[0], itDatabase->second.position[1], itDatabase->second.position[2]);
				User1Right = 7;
				User1Right_X = itDatabase->second.position[0];
				User1Right_Y = itDatabase->second.position[1];
				User1Right_Z = itDatabase->second.position[2];
				// printf("\tPose: %f, %f, %f, %f\n", itDatabase->second.pose[0], itDatabase->second.pose[1], itDatabase->second.pose[2], itDatabase->second.pose[3]);
				// printf("\tHeading(Y): %f\n", itDatabase->second.headingY);
				User1Right_Orientation = itDatabase->second.headingY;
			}
			
			std::string User2Left = "User2Left";
			if (strcmp(itDatabase->first.c_str(), User2Left.c_str()) == 0) {
				// printf("ID: %s:\n", itDatabase->first.c_str());
				// printf("\tPosition: %f, %f, %f\n", itDatabase->second.position[0], itDatabase->second.position[1], itDatabase->second.position[2]);
				User2Left = 8;
				User2Left_X = itDatabase->second.position[0];
				User2Left_Y = itDatabase->second.position[1];
				User2Left_Z = itDatabase->second.position[2];
				// printf("\tPose: %f, %f, %f, %f\n", itDatabase->second.pose[0], itDatabase->second.pose[1], itDatabase->second.pose[2], itDatabase->second.pose[3]);
				// printf("\tHeading(Y): %f\n", itDatabase->second.headingY);
				User2Left_Orientation = itDatabase->second.headingY;
			}
			
			std::string User2Right = "User2Right";
			if (strcmp(itDatabase->first.c_str(), User2Right.c_str()) == 0) {
				// printf("ID: %s:\n", itDatabase->first.c_str());
				// printf("\tPosition: %f, %f, %f\n", itDatabase->second.position[0], itDatabase->second.position[1], itDatabase->second.position[2]);
				User2Right = 9;
				User2Right_X = itDatabase->second.position[0];
				User2Right_Y = itDatabase->second.position[1];
				User2Right_Z = itDatabase->second.position[2];
				// printf("\tPose: %f, %f, %f, %f\n", itDatabase->second.pose[0], itDatabase->second.pose[1], itDatabase->second.pose[2], itDatabase->second.pose[3]);
				// printf("\tHeading(Y): %f\n", itDatabase->second.headingY);
				User2Right_Orientation = itDatabase->second.headingY;
			}
		}
	// } End of while (!_kbhit())
}

void main(void) {

	// PhaseSpaceTracking
	WSADATA wsaDataPhaseSpace;
	WSAStartup(MAKEWORD(2, 2), &wsaDataPhaseSpace);
	sockPhaseSpace = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	char broadcast = 'a';

	if (setsockopt(sockPhaseSpace, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0)
	{
		perror("broadcast options");
		// _getch();
		closesocket(sockPhaseSpace);
		return;
	}

	recv_addr.sin_family = AF_INET;
	recv_addr.sin_port = htons(PORT);
	recv_addr.sin_addr.s_addr = INADDR_ANY;

	if (bind(sockPhaseSpace, (sockaddr*)&recv_addr, sizeof(recv_addr)) < 0)
	{
		perror("Error binding UDP socket.");
		// _getch();
		closesocket(sockPhaseSpace);
		return;
	}

	//Create driver and connect to it
	AsierInhoUDP::RegisterPrintFuncs(print, print, print);
	AsierInhoUDP::AsierInhoUDPBoard* driver = AsierInhoUDP::createAsierInho();

	float distance = 0.10f; // distance between the boards
	int numTransducers = 64 * numPatBoards;
	int boardIDs[] = { 2, 4 }; // board IDs to be connected
	float matBoardToWorld[32] = { // 
		/*left*/
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
		/*right*/
		-1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0,-1, distance,
		0, 0, 0, 1,
	};
	// Connect to the clients
	int socketPort = 54007; // 7777;
	char serverIP[] = "192.168.0.119"; // PC as a Server  

	char PAT_addresses[numPatBoards][14] = {
		// "192.168.0.124", // PAT Board 6 East
		// "192.168.0.176" // PAT Board 7 East
		"192.168.0.139", // PAT Board 2 East
	    "192.168.0.144", // PAT Board 4 East
		// "192.168.0.201",
		// "192.168.0.203",
		// "192.168.0.204",
		// "192.168.0.205",
		// "192.168.0.208",
		// "192.168.0.209",
		// "192.168.0.110"
	};

	char Mona_addresses[numMonaRobots][14] = {
		// "192.168.0.122", // AcoustoBot 2 East
		// "192.168.0.222", // MONA 2 East
		// "192.168.0.223", // AcoustoBot 3
		// "192.168.0.224",  // MONA 4 East
		// "192.168.0.244", // Bead Dispenser 1
		"192.168.0.146" // Bead Dispenser // UCL East // RED

	};

	udp.initialise(serverIP, socketPort);

	for (int i = 0; i < numPatBoards; i++) {
		PATs[i] = udp.addClient(PAT_addresses[i]);
	}

	for (int i = 0; i < numMonaRobots; i++) {
		Monas[i] = udp.addClient(Mona_addresses[i]);
	}

	//Create solver:
	GSPAT_V2::RegisterPrintFuncs(print, print, print);
	GSPAT::Solver* solver = GSPAT_V2::createSolver(numTransducers);//Number of transducers used (two boards of 8x8)
	if (!driver->connect(numPatBoards, boardIDs, matBoardToWorld))	//Device IDs to connect to
		printf("Failed to connect to board.");

	float transducerPositions[128 * 3], transducerNormals[128 * 3], amplitudeAdjust[128];
	int mappings[128], phaseDelays[128], numDiscreteLevels;
	driver->readParameters(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, &numDiscreteLevels);
	solver->setBoardConfig(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, numDiscreteLevels);

	//Program: Create a trap and move it with the keyboard
	const size_t numPoints = 1; //Change this if you want (but make sure to change also each point's position )
	const size_t numGeometries = 1;//Please, do not change this. Multiple update Messages require AsierInhoV2 (see 6.simpleGSPATCpp_AsierInhoV2)
	static float radius = 0.001f; //  0.00865f * 1.5f;// 0.02f;;
	float curPos[4 * numGeometries * numPoints]; // current positions of the levitation/focusing points
	float amplitude[numGeometries * numPoints];	 // target amplitudes of the points
	for (int g = 0; g < numGeometries; g++) {
		for (int p = 0; p < numPoints; p++) {
			float angle = 2 * M_PI / numPoints;
			if (p == 0)
				curPos[4 * g * numPoints + 4 * p + 0] = 0.015f; //radius * cos(p * angle);
			else
				curPos[4 * g * numPoints + 4 * p + 0] = -0.015f; //radius * cos(p * angle);
			curPos[4 * g * numPoints + 4 * p + 1] = 0.0f; // radius* sin(p * angle);
			curPos[4 * g * numPoints + 4 * p + 2] = 0.05f; // distance / 2.f;
			curPos[4 * g * numPoints + 4 * p + 3] = 1;
			amplitude[g * numPoints + p] = 40000;
		}
	}
	unsigned char* msg;
	float mI[] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	float matrix[16 * numPoints];
	for (int i = 0; i < numPoints; i++)
		memcpy(&matrix[16 * i], mI, 16 * sizeof(float));
	//a. create a solution
	GSPAT::Solution* solution = solver->createSolution(numPoints, numGeometries, true, curPos, amplitude, matrix, matrix);
	solver->compute(solution);
	solution->finalMessages(&msg);

	for (int i = 0; i < numPatBoards; i++)
		udp.sendString(&msg[packetSize * 0], packetSize, PATs[i]); // Fill FPGA buffers so update is processed directly
	solver->releaseSolution(solution);
	printf("\n Bead location (%f,%f,%f)\n ", curPos[0], curPos[1], curPos[2]);

	bool finished = false;
	bool commit = false;

	// int bufSize = 9;
	// unsigned char buf[9];

	while (!finished) {
		// Update 3D position from keyboard presses
		// printf("PhaseSpaceTrackingSystem!\n");
		//recvFunct(NULL);

		//if (Bot1 == 1) {
		//	printf("\t Bot Id: %d, Bot Position: (%f, %f, %f) \t Orientation: %f\n", Bot1, Bot1_X, Bot1_Y, Bot1_Z, Bot1_Orientation);
		//	unsigned char* writePointer = &buf[0];
		//	memcpy((void*)writePointer, (void*)&Bot1, sizeof(int)); writePointer += sizeof(int);

		//	memcpy((void*)writePointer, (void*)&Bot1_X, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot1_Y, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot1_Z, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot1_Orientation, sizeof(float)); writePointer += sizeof(float);

		//	memcpy((void*)writePointer, (void*)&Bot1_X_d, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot1_Y_d, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot1_Z_d, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot1_Orientation_d, sizeof(float)); writePointer += sizeof(float);

		//	udp.sendString(&buf[0], 36, Monas[0]);
		//}
		//
		//if (Bot2 == 2) {
		//	printf("\t Bot Id: %d, Bot Position: (%f, %f, %f) \t Orientation: %f\n", Bot2, Bot2_X, Bot2_Y, Bot2_Z, Bot2_Orientation);
		//	unsigned char* writePointer = &buf[0];
		//	memcpy((void*)writePointer, (void*)&Bot2, sizeof(int)); writePointer += sizeof(int);

		//	memcpy((void*)writePointer, (void*)&Bot2_X, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot2_Y, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot2_Z, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot2_Orientation, sizeof(float)); writePointer += sizeof(float);

		//	memcpy((void*)writePointer, (void*)&Bot2_X_d, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot2_Y_d, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot2_Z_d, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot2_Orientation_d, sizeof(float)); writePointer += sizeof(float);

		//	udp.sendString(&buf[0], 36, Monas[1]);
		//}
		//
		//if (Bot3 == 3) {
		//	printf("\t Bot Id: %d, Bot Position: (%f, %f, %f) \t Orientation: %f\n", Bot3, Bot3_X, Bot3_Y, Bot3_Z, Bot3_Orientation);
		//	unsigned char* writePointer = &buf[0];
		//	memcpy((void*)writePointer, (void*)&Bot3, sizeof(int)); writePointer += sizeof(int);

		//	memcpy((void*)writePointer, (void*)&Bot3_X, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot3_Y, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot3_Z, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot3_Orientation, sizeof(float)); writePointer += sizeof(float);

		//	memcpy((void*)writePointer, (void*)&Bot3_X_d, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot3_Y_d, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot3_Z_d, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot3_Orientation_d, sizeof(float)); writePointer += sizeof(float);

		//	udp.sendString(&buf[0], 36, Monas[2]);
		//}
		//
		//if (Bot4 == 4) {
		//	printf("\t Bot Id: %d, Bot Position: (%f, %f, %f) \t Orientation: %f\n", Bot4, Bot4_X, Bot4_Y, Bot4_Z, Bot4_Orientation);
		//	unsigned char* writePointer = &buf[0];
		//	memcpy((void*)writePointer, (void*)&Bot4, sizeof(int)); writePointer += sizeof(int);

		//	memcpy((void*)writePointer, (void*)&Bot4_X, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot4_Y, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot4_Z, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot4_Orientation, sizeof(float)); writePointer += sizeof(float);

		//	memcpy((void*)writePointer, (void*)&Bot4_X_d, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot4_Y_d, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot4_Z_d, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot4_Orientation_d, sizeof(float)); writePointer += sizeof(float);

		//	udp.sendString(&buf[0], 36, Monas[3]);
		//}

		//if (Bot5 == 5) {
		//	printf("\t Bot Id: %d, Bot Position: (%f, %f, %f) \t Orientation: %f\n", Bot5, Bot5_X, Bot5_Y, Bot5_Z, Bot5_Orientation);
		//	unsigned char* writePointer = &buf[0];
		//	memcpy((void*)writePointer, (void*)&Bot5, sizeof(int)); writePointer += sizeof(int);
		//
		//	memcpy((void*)writePointer, (void*)&Bot5_X, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot5_Y, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot5_Z, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot5_Orientation, sizeof(float)); writePointer += sizeof(float);

		//	memcpy((void*)writePointer, (void*)&Bot5_X_d, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot5_Y_d, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot5_Z_d, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&Bot5_Orientation_d, sizeof(float)); writePointer += sizeof(float);
		//
		//	udp.sendString(&buf[0], 36, Monas[4]);
		//}

		//if (User1Left == 6) {
		//	printf("\t User Id: %d, User Position: (%f, %f, %f) \t Orientation: %f\n", User1Left, User1Left_X, User1Left_Y, User1Left_Z, User1Left_Orientation);
		//	unsigned char* writePointer = &buf[0];
		//	memcpy((void*)writePointer, (void*)&User1Left, sizeof(int)); writePointer += sizeof(int);
		//	
		//	memcpy((void*)writePointer, (void*)&User1Left_X, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&User1Left_Y, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&User1Left_Z, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&User1Left_Orientation, sizeof(float)); writePointer += sizeof(float);

		//	// udp.sendString(&buf[0], 20, Monas[5]);
		//}

		//if (User1Right == 7) {
		//	printf("\t User Id: %d, User Position: (%f, %f, %f) \t Orientation: %f\n", User1Right, User1Right_X, User1Right_Y, User1Right_Z, User1Right_Orientation);
		//	unsigned char* writePointer = &buf[0];
		//	memcpy((void*)writePointer, (void*)&User1Right, sizeof(int)); writePointer += sizeof(int);
		//	
		//	memcpy((void*)writePointer, (void*)&User1Right_X, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&User1Right_Y, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&User1Right_Z, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&User1Right_Orientation, sizeof(float)); writePointer += sizeof(float);

		//	// udp.sendString(&buf[0], 20, Monas[6]);
		//}

		//if (User2Left == 8) {
		//	printf("\t User Id: %d, User Position: (%f, %f, %f) \t Orientation: %f\n", User2Left, User2Left_X, User2Left_Y, User2Left_Z, User2Left_Orientation);
		//	unsigned char* writePointer = &buf[0];
		//	memcpy((void*)writePointer, (void*)&User2Left, sizeof(int)); writePointer += sizeof(int);
		//	
		//	memcpy((void*)writePointer, (void*)&User2Left_X, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&User2Left_Y, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&User2Left_Z, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&User2Left_Orientation, sizeof(float)); writePointer += sizeof(float);

		//	// udp.sendString(&buf[0], 20, Monas[5]);
		//}

		//if (User2Right == 9) {
		//	printf("\t User Id: %d, User Position: (%f, %f, %f) \t Orientation: %f\n", User2Right, User2Right_X, User2Right_Y, User2Right_Z, User2Right_Orientation);
		//	unsigned char* writePointer = &buf[0];
		//	memcpy((void*)writePointer, (void*)&User2Right, sizeof(int)); writePointer += sizeof(int);
		//	
		//	memcpy((void*)writePointer, (void*)&User2Right_X, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&User2Right_Y, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&User2Right_Z, sizeof(float)); writePointer += sizeof(float);
		//	memcpy((void*)writePointer, (void*)&User2Right_Orientation, sizeof(float)); writePointer += sizeof(float);

		//	// udp.sendString(&buf[0], 20, Monas[6]);
		//}

		switch (_getch()) {
			case 'a':
				for (int g = 0; g < numGeometries; g++)
					for (int p = 0; p < numPoints; p++)
						curPos[4 * g * numPoints + 4 * p + 0] += 0.0005f;
				printf("switch case a! \n");
				commit = true; break;
			case 'd':
				for (int g = 0; g < numGeometries; g++)
					for (int p = 0; p < numPoints; p++)
						curPos[4 * g * numPoints + 4 * p + 0] -= 0.0005f;
				printf("switch case d! \n");
				commit = true; break;
			case 'w':
				for (int g = 0; g < numGeometries; g++)
					for (int p = 0; p < numPoints; p++)
						curPos[4 * g * numPoints + 4 * p + 1] += 0.0005f;
				printf("switch case w! \n");
				commit = true; break;
			case 's':
				for (int g = 0; g < numGeometries; g++)
					for (int p = 0; p < numPoints; p++)
						curPos[4 * g * numPoints + 4 * p + 1] -= 0.0005f;
				printf("switch case s! \n");
				commit = true; break;
			case 'q':
				for (int g = 0; g < numGeometries; g++)
					for (int p = 0; p < numPoints; p++)
						curPos[4 * g * numPoints + 4 * p + 2] += 0.00025f;
				commit = true; break;
			case 'e':
				for (int g = 0; g < numGeometries; g++)
					for (int p = 0; p < numPoints; p++)
						curPos[4 * g * numPoints + 4 * p + 2] -= 0.00025f;
				commit = true; break;
			case 'F':
				for (int i = 0; i < numMonaRobots; i++)
					udp.sendByte('F', Monas[i]);

				break;
			case 'B':
				for (int i = 0; i < numMonaRobots; i++)
					udp.sendByte('B', Monas[i]);
				break;
			case 'L':
				for (int i = 0; i < numMonaRobots; i++)
					udp.sendByte('L', Monas[i]);
				break;
			case 'R':
				for (int i = 0; i < numMonaRobots; i++)
					udp.sendByte('R', Monas[i]);
				break;
			case 'H':
				for (int i = 0; i < numMonaRobots; i++)
					udp.sendByte('H', Monas[i]);
				printf("Hinge Activated! \n");
				break;
			case 'D':
				for (int i = 0; i < numMonaRobots; i++)
					udp.sendByte('D', Monas[i]);
				printf("Hinge De-activated! \n");
				break;
			case 'b':
				for (int i = 0; i < numMonaRobots; i++)
					udp.sendByte('b', Monas[i]);
				printf("Bead Dispenser Activated! \n");
				break;
			case '1':
				commit = false; 
				break;
			case 'c':
				// printf("\t!!!Inside Loop Position: %f, %f, %f\n", X, Y, Z);
				// printf("\tPose: %f, %f, %f, %f\n", itDatabase->second.pose[0], itDatabase->second.pose[1], itDatabase->second.pose[2], itDatabase->second.pose[3]);
				// printf("\tHeading(Y): %f\n", Heading);
				// for (int i = 0; i < numMonaRobots; i++)
					// udp.sendByte('c', Monas[i]);
				break;
			case 'x':
			case 'X':
				finished = true;
		} 

		if (commit) {
			GSPAT::Solution* newSolution = solver->createSolution(numPoints, numGeometries, true, curPos, amplitude, matrix, matrix);
			solver->compute(newSolution);
			newSolution->finalMessages(&msg);
			for (int i = 0; i < numPatBoards; i++)
				udp.sendString(&msg[packetSize * 0], packetSize, PATs[i]);
			solver->releaseSolution(newSolution);
			printf("\n Bead location (%f,%f,%f)\n ", curPos[0], curPos[1], curPos[2]);
			commit = false;
		}
	}

	closesocket(sockPhaseSpace);
	WSACleanup();
	
	// udp.disconnect();
	//Deallocate the AsierInho controller: 
    driver->turnTransducersOff();
	Sleep(100);
	driver->disconnect();
	delete driver;
	delete solver;
}