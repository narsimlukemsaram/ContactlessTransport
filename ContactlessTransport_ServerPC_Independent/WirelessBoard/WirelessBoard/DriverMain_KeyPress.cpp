// Program:	Create a focal point using the "createTrap" function and move it with the keyboard
//			This program would be a great starting point to learn how to use the driver
//			(The driver enables us to communicate to phased arrays of transducers, PATs) 

#include <src/UDPDriver.h>
#include <AsierInhoUDP.h>
#include <AsierInhoSerial.h>
#include <GSPAT_SolverV2.h>
#include <Helper\HelperMethods.h>
#include <Helper\microTimer.h>
#include <string.h>
#include <conio.h>

UDPDriver udp;
int packetSize = 128;
const int numPatBoards = 1; // number of PAT boards
const int numMonaRobots = 1; // number of MONA robots
int PATs[numPatBoards], Monas[numMonaRobots];

void print(const char* str) {
	printf("%s\n", str);
}

// this function allows to create a focal point
void createTrap(float position[3], int numTransducers, float *t_positions, float* amplitudes, float* phases, bool levitationSignature = false) {
	float pitch = 0.0105f;
	int buffer_index = 0;
	//1. Traverse each transducer:
	for (buffer_index = 0; buffer_index < numTransducers; buffer_index++)
	{
		//a. Get its position:
		float* t_pos = &(t_positions[3 * buffer_index]);
		//b. Compute phase delay and amplitudes from transducer to target point: 
		float distance, amplitude, signature;
		computeAmplitudeAndDistance(t_pos, position, &amplitude, &distance);
		signature = (t_pos[0] > 0.f ? (float)M_PI : 0);//Add PI to transducers on the half of the transducers (e.g. only the top board).
		float phase = fmodf(-K() * distance + signature * (float)(levitationSignature), (float)(2 * M_PI));

		//c. Store it in the output buffers.
		phases[buffer_index] = phase;
		amplitudes[buffer_index] = 1.f;// amplitude;
	}
}

void main(void) {
	//1. Prepare your driver and solver
	//1.1. Create driver and connect to it
	//1.1.a These two lines are for communicating the boards through WiFi (UDP protocol)
	AsierInhoUDP::RegisterPrintFuncs(print, print, print);
	AsierInhoUDP::AsierInhoUDPBoard* driver = AsierInhoUDP::createAsierInho();
	//1.1.b These two lines are for communicating the boards through Bluetooth/Serial communication
	//AsierInhoSerial::RegisterPrintFuncs(print, print, print);
	//AsierInhoSerial::AsierInhoSerialBoard* driver = AsierInhoSerial::createAsierInho();
	//1.2. Setting the board
	// float distance = -0.084; // distance between the boards
    // const int numTransducers = 128;
	// const int numTransducers = 64;
	int numBoards = 1; // how many boards do you use?
	int boardIDs[] = { 1 }; // board ID for Wifi, but COM port number for Serial/Bluetooth
	float matBoardToWorld[] = { // only using the top board in this example
		/*first*/
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
		/*second*/
		1, 0, 0, 0,
		0, 1, 0, -0.084,
		0, 0, 1, 0,
		0, 0, 0, 1,
	};
	// 1.2. Connect to the boards and get some configurations
	int socketPort = 54007;
	char serverIP[] = "192.168.0.188"; // PC as a Server  

	char PAT_addresses[numPatBoards][14] = {
		"192.168.0.181", // PAT Board (Yellow) Master
		// "192.168.0.127",   // PAT Board (Green) Slave
		// "192.168.0.141", // PAT Board 3 (Red) for Single-Sided Levitation
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
		"192.168.0.195", // Acoustic Robot (Yellow) Leader
		// "192.168.0.196" // Acoustic Robot (Green) Follower
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
	GSPAT::Solver* solver = GSPAT_V2::createSolver(128); // Number of transducers used (two boards of 8x8)
	if (!driver->connect(numPatBoards, boardIDs, matBoardToWorld)) //Device IDs to connect to
		printf("Failed to connect to board.");
	float transducerPositions[128 * 3], transducerNormals[128 * 3], amplitudeAdjust[128];
	int mappings[128], phaseDelays[128], numDiscreteLevels;
	driver->readParameters(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, &numDiscreteLevels);
	
	// 2. Define your focal point position
	float curPos[] = { 0, 0, 0.03f }; // { 0, 0, 0.06f }; // position of the focal point
	bool twinTrap = false; // converting the focal point to a twin trap by applying the levitation signature
	const int numTransducers = 64;
	float phases[numTransducers], amplitudes[numTransducers]; // phases and amplitudes to be computed
	//2.1. create a first trap and send it to the board: 
	createTrap(curPos, numTransducers, transducerPositions, amplitudes, phases, twinTrap);
	//2.2. discretize and send the computed phases and amplitudes
	unsigned char* message = driver->discretize(phases, amplitudes, numDiscreteLevels);
	driver->updateMessage(message);

	//3. Main loop (finished when the key X is pressed):
	bool commit = false;
	bool finished = false;
	printf("\n The current point position is at (%f,%f,%f)\n ", curPos[0], curPos[1], curPos[2]);
	printf("Use keys A-D , W-S and Q-E to move the point\n");
	printf("Press 'X' to destroy the solver.\n");
	while (!finished) {
		//3.1. Update the 3D positions from keyboard presses
		switch (_getch()) {
		case 'a':
			curPos[0] += 0.0005f; commit = true; break;
		case 'd':
			curPos[0] -= 0.0005f; commit = true; break;
		case 'w':
			curPos[1] += 0.0005f; commit = true; break;
		case 's':
			curPos[1] -= 0.0005f; commit = true; break;
		case 'q':
			curPos[2] += 0.00025f; commit = true; break;
		case 'e':
			curPos[2] -= 0.00025f; commit = true; break;
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
			//2.2. create a first trap and send it to the board: 
			createTrap(curPos, numTransducers, transducerPositions, amplitudes, phases, twinTrap);
			//2.3. discretize and send the computed phases and amplitudes
			unsigned char* message = driver->discretize(phases, amplitudes, numDiscreteLevels);
			driver->updateMessage(message);
			//3.4. print the current position
			commit = false;
			printf("\n Current point location (%f,%f,%f)\n ", curPos[0], curPos[1], curPos[2]);
		}
	}
	//5. Deallocate the AsierInho controller: 
	driver->turnTransducersOff();
	Sleep(100);
	driver->disconnect();
	delete driver;
}
