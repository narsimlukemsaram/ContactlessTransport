#include <src/UDPDriver.h>
#include <AsierInhoUDP.h>
#include <GSPAT_SolverV2.h>
#include <Helper\HelperMethods.h>
#include <Helper\microTimer.h>
#include <string.h>
#include <conio.h>

UDPDriver udp;
int packetSize = 128;
const int numPatBoards = 2; // number of PAT boards
const int numMonaRobots = 2; // number of MONA robots
int PATs[numPatBoards], Monas[numMonaRobots];

void print(const char* str) {
	printf("%s\n", str);
}

void updatePhasesAndAmplitudes(AsierInhoUDP::AsierInhoUDPBoard* driver, float* phases, float* amplitudes) {
	unsigned char* message = driver->discretize(phases, amplitudes);
	driver->updateMessage(message);
}


void main(void) {
	//Create driver and connect to it
	AsierInhoUDP::RegisterPrintFuncs(print, print, print);
	AsierInhoUDP::AsierInhoUDPBoard* driver = AsierInhoUDP::createAsierInho();
	float distance = 0.10f; // distance between the boards
	int numTransducers = 64 * numPatBoards;
	int boardIDs[] = { 1, 2 }; // board IDs to be connected
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
	char serverIP[] = "192.168.0.188"; // PC as a Server  

	char PAT_addresses[numPatBoards][14] = {
		"192.168.0.181", // PAT Board 1 (Yellow) Master
		"192.168.0.127", // PAT Board 2 (Green) Slave
		// "192.168.0.139", // PAT Board 2 East Red
		// "192.168.0.144", // PAT Board 4 East Blue
		// "192.168.0.201",
		// "192.168.0.203",
		// "192.168.0.204",
		// "192.168.0.205",
		// "192.168.0.208",
		// "192.168.0.209",
		// "192.168.0.110"
	};
	char Mona_addresses[numMonaRobots][14] = {
		"192.168.0.195", // Acoustic Robot 1 (Yellow) Leader
		"192.168.0.191" // Acoustic Robot 2 (Green) Follower
		// "192.168.0.181", // Mona Robot 19 (AcoustoBot 3 Red)
		// "192.168.0.195",  // Mona Robot 4 (AcoustoBot 4 Blue)
		// "192.168.0.149" // Mona Robot 5 (Bead Dispenser 1 Red)
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
	static float radius = 0.001f; //  0.00865f * 1.5f;// 0.02f;
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

	while (!finished) {
		//Update 3D position from keyboard presses
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
				commit = false; break;
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
	// udp.disconnect();
	//Deallocate the AsierInho controller: 
    driver->turnTransducersOff();
	Sleep(100);
	driver->disconnect();
	delete driver;
	delete solver;
}