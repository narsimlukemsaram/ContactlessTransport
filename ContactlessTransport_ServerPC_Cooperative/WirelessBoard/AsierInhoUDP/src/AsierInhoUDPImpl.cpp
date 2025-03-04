#include <src/AsierInhoUDPImpl.h>
#include <sstream>
#include <fstream>


static char consoleLineBuffer[512];

/**
	Method run by each of the wroker threads
*/
void* worker_BoardUpdater(void* threadData/*, void* driver*/) {
	//we read ou configuration
	struct worker_threadData* data = (struct worker_threadData*)threadData;
	AsierInhoUDPImpl* asierInho = (AsierInhoUDPImpl*)data->thisPointer;

	
	while (data->running) {
		//Wait for our signal
		pthread_mutex_lock(&data->send_signal);
		//Send update:
		pthread_mutex_lock(&(asierInho->boardUsed));
		AsierInhoUDPImpl::_sendUpdate(asierInho->board, data->dataStream, data->numMessagesToSend, data->messageSize, data->clientID);
		pthread_mutex_unlock(&(asierInho->boardUsed));
		data->numMessagesToSend = 0;
		//Send notification signal
		pthread_mutex_unlock(&data->done_signal);		
	}	
	AsierInhoUDP::printMessage("AsierInho Worker finished!\n");
	return 0;
}

AsierInhoUDPImpl::AsierInhoUDPImpl() :AsierInhoUDPBoard(), status(INIT) {
	
}

AsierInhoUDPImpl::~AsierInhoUDPImpl() {
	disconnect();
	//delete topData;
	//delete bottomData;
	for (int b = 0; b < boardWorkerData.size(); b++)
		delete boardWorkerData[b];
	boardWorkerData.clear();
}

/**
	Connects to the board, using the board type and ID specified.
	ID corresponds to the index we labeled on each board.
	Returns true if connection was succesfull.
*/
bool AsierInhoUDPImpl::connect(int bottomBoardID, int topBoardID, int maxNumMessagesToSend) {
	//Configuration data for a simple top-bottom setup.
	int boardIDs[] = { bottomBoardID, topBoardID };
	float matBoardToWorld[32] = {	/*bottom*/
								1, 0, 0, 0,
								0, 1, 0, 0,
								0, 0, 1, 0,
								0, 0, 0, 1,	
								/*top*/
								-1, 0, 0, 0,
								0, 1, 0, 0,
								0, 0,-1, 0.24f,
								0, 0, 0, 1,	
	};
	if (topBoardID != 0)//Support for sigle sided setups
		return connect(2, boardIDs, matBoardToWorld, maxNumMessagesToSend);
	else
		return connect(1, boardIDs, matBoardToWorld, maxNumMessagesToSend);
}

bool AsierInhoUDPImpl::connect(int numBoards, int * boardIDs, float* matToWorld , int maxNumMessagesToSend)
{
	BoardConfig boardConfig = ParseBoardConfig::readParameters(std::stringstream("board_mini.pat"));
	readIPConfig("ip_addresses.pat");
	this->numBoards = numBoards;
	this->maxNumMessagesToSend = maxNumMessagesToSend;
	this->numTransducersPerBoard = boardConfig.numTransducers;
	this->messageSize = this->numTransducersPerBoard * 2;

	//1. Create resources for worker threads: 
	dataStream = new unsigned char[messageSize*numBoards*maxNumMessagesToSend];
	transducerPositions = new float[numTransducersPerBoard * 3 * numBoards];
	transducerNormals = new float[numTransducersPerBoard * 3 * numBoards];
	amplitudeAdjust= new float[numTransducersPerBoard * numBoards];
	phaseAdjust = new int[numTransducersPerBoard * numBoards];
	transducerIds = new int [numTransducersPerBoard *numBoards];
	int* numDiscreteLevels = new int [numBoards];
	this->numDiscreteLevels = 256; //We will set the resolution to the MINIMUM of all board involved
	//2. Read per-board adjustment parameters and safe them in each worker thread data:
	for (int b = 0; b < numBoards; b++) {
		this->boardIDs.push_back(boardIDs[b]);	
		boardWorkerData.push_back(new struct worker_threadData);
		boardWorkerData[b]->dataStream = &(dataStream[b*messageSize*maxNumMessagesToSend]);
		//readBoardParameters(boardIDs[b], &(matToWorld[16*b]), &(transducerPositions[3*256*b]), &(transducerNormals[3 * 256 * b]), &(transducerIds[256*b]), &(phaseAdjust[256*b]), &(amplitudeAdjust[256*b]), &numDiscreteLevels[b], (boardWorkerData[b]));
		copyBoardParameters(boardConfig, boardIDs[b], &(matToWorld[16 * b]), &(transducerPositions[3 * numTransducersPerBoard * b]), &(transducerNormals[3 * numTransducersPerBoard * b]), &(transducerIds[numTransducersPerBoard * b]), &(phaseAdjust[numTransducersPerBoard * b]), &(amplitudeAdjust[numTransducersPerBoard * b]), &numDiscreteLevels[b], (boardWorkerData[b]));
		if (numDiscreteLevels[b] < this->numDiscreteLevels)
			this->numDiscreteLevels = numDiscreteLevels[b];
	}
	delete numDiscreteLevels;

	//Adjust from (local) transducer IDs [0..255] to global positions in the message 
	{
		for (int b = 1; b < numBoards; b++)
			for (int t = 0; t < numTransducersPerBoard; t++)
				this->transducerIds[t + b * numTransducersPerBoard] += b * numTransducersPerBoard;
	}
	//3. Connect to each of the boards:
	if (status == AsierInhoState::INIT) {
		bool allConnected = initUDPDriver();
		pthread_mutex_init(&(boardUsed), NULL);

		if (allConnected){
			status = AsierInhoState::CONNECTED;
			for (int b = 0; b < numBoards; b++) {
				pthread_mutex_init(&(boardWorkerData[b]->send_signal), NULL);
				pthread_mutex_lock(&(boardWorkerData[b]->send_signal));
				pthread_mutex_init(&(boardWorkerData[b]->done_signal), NULL);
				pthread_attr_t attrs;
				pthread_attr_init(&attrs);
				pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_JOINABLE);
				pthread_t thread; 
				boardWorkerThreads.push_back(thread);
				boardWorkerData[b]->thisPointer = (void*)this;
				pthread_create(&boardWorkerThreads[b], &attrs, &worker_BoardUpdater, ((void*)boardWorkerData[b]));
			
			}			
			Sleep(100);
			AsierInhoUDP::printMessage("AsierInhoUDP connected succesfully\n");
			return true;
		}
		else {
			//Destroy connections created? (0..boardConnected)
			AsierInhoUDP::printWarning("AsierInhoUDP connection failed\n");
			return false;
		}
	}
	AsierInhoUDP::printWarning("AsierInhoUDP is already connected\n");
	return false;
}

void AsierInhoUDPImpl::readParameters(float * transducerPositions, float* transducerNormals, int * transducerIds, int * phaseAdjust, float * amplitudeAdjust, int * numDiscreteLevels)
{
	memcpy(transducerPositions, this->transducerPositions, numTransducersPerBoard * numBoards * 3 * sizeof(float));
	memcpy(transducerNormals, this->transducerNormals, numTransducersPerBoard * numBoards * 3 * sizeof(float));
	memcpy(transducerIds , this->transducerIds, numTransducersPerBoard *numBoards*sizeof(int));
	memcpy(phaseAdjust , this->phaseAdjust, numTransducersPerBoard *numBoards*sizeof(int));
	memcpy(amplitudeAdjust , this->amplitudeAdjust, numTransducersPerBoard *numBoards*sizeof(float));
	*numDiscreteLevels = this->numDiscreteLevels;
}

void AsierInhoUDPImpl::updateBoardPositions(float * matBoardToWorld4x4)
{
	//Pause worker threads 
	if(status==CONNECTED)
		for (int b = 0; b < numBoards; b++) 
			pthread_mutex_lock(&(boardWorkerData[b]->done_signal));
	//Update configuration (we simlpy re-read the config file, applying the new matrix).
	int aux;//we do not need to reload the numDiscreteLevels... we will store it here and ignore this return value
	for (int b = 0; b < numBoards; b++) 
		readBoardParameters(boardIDs[b], &(matBoardToWorld4x4[16*b]), &(transducerPositions[3*256*b]), &(transducerNormals[3*256*b]), &(transducerIds[256 * b]), &(phaseAdjust[256 * b]), &(amplitudeAdjust[256 * b]), &aux, (boardWorkerData[b]));
	//Unlock worker threads
	if(status==CONNECTED)
		for (int b = 0; b < numBoards; b++) 
			pthread_mutex_unlock(&(boardWorkerData[b]->done_signal));
}

void AsierInhoUDPImpl::readBoardParameters(int boardId, float* matToWorld, float* transducerPositions, float* transducerNormals, int* transducerIds, int* phaseAdjust, float* amplitudeAdjust, int* numDiscreteLevels, worker_threadData* thread_data) {
	BoardConfig boardConfig = ParseBoardConfig::readParameters(boardId);
	//thread_data->boardSN = new char[strlen(boardConfig.hardwareID) + 1];
	//strcpy_s(thread_data->boardSN, strlen(boardConfig.hardwareID) + 1, boardConfig.hardwareID);
	//0. Num discrete levels
	*numDiscreteLevels = boardConfig.numDiscreteLevels;
	//1. Write transducer positions (multiply local pos by matrix): 
	for (int t = 0; t < 256; t++) {
		float* pLocal = &(boardConfig.positions[3 * t]);
		float posWorld[3] = { pLocal[0] * matToWorld[0] + pLocal[1] * matToWorld[1] + pLocal[2] * matToWorld[2] + matToWorld[3],
							 pLocal[0] * matToWorld[4] + pLocal[1] * matToWorld[5] + pLocal[2] * matToWorld[6] + matToWorld[7],
							 pLocal[0] * matToWorld[8] + pLocal[1] * matToWorld[9] + pLocal[2] * matToWorld[10] + matToWorld[11], };
		memcpy(&(transducerPositions[3 * t]), posWorld, 3 * sizeof(float));
		float nLocal[3] = { 0, 0, 1.f };
		float normWorld[3] = { nLocal[0] * matToWorld[0] + nLocal[1] * matToWorld[1] + nLocal[2] * matToWorld[2],
							   nLocal[0] * matToWorld[4] + nLocal[1] * matToWorld[5] + nLocal[2] * matToWorld[6],
							   nLocal[0] * matToWorld[8] + nLocal[1] * matToWorld[9] + nLocal[2] * matToWorld[10], };
		memcpy(&(transducerNormals[3 * t]), normWorld, 3 * sizeof(float));
	}
	//2. Write transducer IDs:
	memcpy(transducerIds, boardConfig.pinMapping, 256 * sizeof(int));
	//3. Write Phase adjustments:
	memcpy(phaseAdjust, boardConfig.phaseAdjust, 256 * sizeof(int));
	//4. Write Amplitude adjustments:
	memcpy(amplitudeAdjust, boardConfig.amplitudeAdjust, 256 * sizeof(float));
}

void AsierInhoUDPImpl::copyBoardParameters(BoardConfig boardConfig, int boardId, float* matToWorld, float* transducerPositions, float* transducerNormals, int* transducerIds, int* phaseAdjust, float* amplitudeAdjust, int* numDiscreteLevels, worker_threadData* thread_data) {
	//thread_data->portNumber = boardId;
	thread_data->messageSize = this->messageSize;
	//0. Num discrete levels
	*numDiscreteLevels = boardConfig.numDiscreteLevels;
	//1. Write transducer positions (multiply local pos by matrix): 
	for (int t = 0; t < numTransducersPerBoard; t++) {
		float* pLocal = &(boardConfig.positions[3 * t]);
		float posWorld[3] = { pLocal[0] * matToWorld[0] + pLocal[1] * matToWorld[1] + pLocal[2] * matToWorld[2] + matToWorld[3],
							 pLocal[0] * matToWorld[4] + pLocal[1] * matToWorld[5] + pLocal[2] * matToWorld[6] + matToWorld[7],
							 pLocal[0] * matToWorld[8] + pLocal[1] * matToWorld[9] + pLocal[2] * matToWorld[10] + matToWorld[11], };
		memcpy(&(transducerPositions[3 * t]), posWorld, 3 * sizeof(float));
		float nLocal[3] = { 0, 0, 1.f };
		float normWorld[3] = { nLocal[0] * matToWorld[0] + nLocal[1] * matToWorld[1] + nLocal[2] * matToWorld[2],
							   nLocal[0] * matToWorld[4] + nLocal[1] * matToWorld[5] + nLocal[2] * matToWorld[6],
							   nLocal[0] * matToWorld[8] + nLocal[1] * matToWorld[9] + nLocal[2] * matToWorld[10], };
		memcpy(&(transducerNormals[3 * t]), normWorld, 3 * sizeof(float));
	}
	//2. Write transducer IDs:
	memcpy(transducerIds, boardConfig.pinMapping, numTransducersPerBoard * sizeof(int));
	//3. Write Phase adjustments:
	memcpy(phaseAdjust, boardConfig.phaseAdjust, numTransducersPerBoard * sizeof(int));
	//4. Write Amplitude adjustments:
	memcpy(amplitudeAdjust, boardConfig.amplitudeAdjust, numTransducersPerBoard * sizeof(float));
}

void AsierInhoUDPImpl::readIPConfig(std::string filename) {
	printf("\n-------------------------------\n");
	printf("IP Configuration List: %s\n", filename.c_str());
	FILE* file;
	fopen_s(&file, filename.c_str(), "r");
	if (!file)
		printf("Could not open ip configuration file\n");
	else {
		fscanf(file, "%d\n", &socketPort);
		printf("%d => Socket port\n", socketPort);

		int IP_index;
		char IP_address[14];
		char line[100];
		while (fgets(line, sizeof(line), file) != NULL) {
			if (line[0] == '/' && line[1] == '/')
				continue; // ignore comment lines

			if (sscanf(line, "%d, %s", &IP_index, IP_address) == 2) {
				char* tmp_address = new char[14];
				strcpy_s(tmp_address, 14, IP_address);

				IPs[IP_index] = tmp_address;
				printf("%2d, %s", IP_index, IP_address);
				if (IP_index == 0) printf(" => Server IP address\n");
				else printf(" => Client IP address\n");
			}
		}
	}
	printf("-------------------------------\n\n");
}

//
bool AsierInhoUDPImpl::initUDPDriver() {
	board.initialise(IPs[0], socketPort);
	for (int b = 0; b < numBoards; b++) {
		boardWorkerData[b]->clientID = board.addClient(IPs[boardIDs[b]]);
		sprintf_s<512>(consoleLineBuffer, "Added Client: %s\n", IPs[boardIDs[b]]);
		AsierInhoUDP::printWarning(consoleLineBuffer);
	}
	
	if (board.anyErrors()) {
		sprintf_s<512>(consoleLineBuffer, "Could not initialise UDP port %s\n", IPs[0]);
		AsierInhoUDP::printWarning(consoleLineBuffer);
		return false;
	}
	else {
		AsierInhoUDP::printMessage("UDP ports initialised\n");
		return true;
	}
}

void AsierInhoUDPImpl::updateMessagePerBoard(unsigned char ** message)
{
	//0. Check status
	if (status != AsierInhoState::CONNECTED )
		return;
	//Multithreaded version: 
	//Wait for them to finish
	for (int b = 0; b < numBoards; b++) {
		pthread_mutex_lock(&(boardWorkerData[b]->done_signal));
		boardWorkerData[b]->numMessagesToSend = 1;
		boardWorkerData[b]->dataStream=&(dataStream[b*messageSize]);
	}
	
	//Update the buffers:
	for (int b = 0; b < numBoards; b++) {
		memcpy(boardWorkerData[b]->dataStream, message[b], messageSize);
	}
	//Send signals to notify worker threads to update phases
	for(int b=0; b<numBoards; b++)
		pthread_mutex_unlock(&(boardWorkerData[b]->send_signal));

}

void AsierInhoUDPImpl::updateMessage(unsigned char* message) {
	//0. Check status
	if (status != AsierInhoState::CONNECTED )
		return;
	//Multithreaded version: 
	//Wait for them to finish
	for (int b = 0; b < numBoards; b++) {
		pthread_mutex_lock(&(boardWorkerData[b]->done_signal));
		boardWorkerData[b]->numMessagesToSend = 1;
		boardWorkerData[b]->dataStream=&(dataStream[b*messageSize]);
	}
	
	//Update the buffer:
	memcpy(dataStream, message, messageSize*numBoards);

	//Send signals to notify worker threads to update phases
	for(int b=0; b<numBoards; b++)
		pthread_mutex_unlock(&(boardWorkerData[b]->send_signal));
}

void AsierInhoUDPImpl::updateMessages(unsigned char* message, int numMessagesToSend) {
	//0. Check status
	if (status != AsierInhoState::CONNECTED)
		return;
	if(numMessagesToSend>maxNumMessagesToSend || numMessagesToSend<1)
		AsierInhoUDP::printWarning("AsierInho: The driver cannot send the requested number of messages. Command Ignored.");
	//Multithreaded version: 
	//Wait for them to finish
	for (int b = 0; b < numBoards; b++) {
		pthread_mutex_lock(&(boardWorkerData[b]->done_signal));
		boardWorkerData[b]->numMessagesToSend = numMessagesToSend;
		boardWorkerData[b]->dataStream = &(dataStream[b*numMessagesToSend*messageSize]);
	}

	//Update the buffer:
	memcpy(dataStream, message, numBoards*numMessagesToSend*messageSize);

	//Send signals to notify worker threads to update phases
	for (int b = 0; b < numBoards; b++)
		pthread_mutex_unlock(&(boardWorkerData[b]->send_signal));
}

/**
	This method turns the transducers off (so that the board does not heat up/die misserably)
	The board is still connected, so it can later be used again (e.g. create new traps)
*/
void AsierInhoUDPImpl::turnTransducersOff() {
	//0. Check status
	if (status != AsierInhoState::CONNECTED)
		return;

	//1. Set all phases and amplitudes to "0"== OFF and send!
	unsigned char* message = new unsigned char[messageSize*numBoards];
	memset(message, 0, messageSize *numBoards* sizeof(unsigned char));	//Set all phases and amplitudes.
	for (int b = 0; b < numBoards; b++) {
		message[0 + messageSize * b] = 0 + 128;	//Set flag indicating new update (>128)
	}
	//2. Send it: 
	updateMessage(message);
	delete message;
}

void AsierInhoUDPImpl::turnTransducersOn() {
	//0. Check status
	if (status != AsierInhoState::CONNECTED)
		return;

	//1. Set all phases and amplitudes to "0"== OFF and send!
	unsigned char* message = new unsigned char[messageSize *numBoards];
	memset(message, 64, messageSize *numBoards* sizeof(unsigned char));	//Set all phases and amplitudes.
	for (int b = 0; b < numBoards; b++) {
		message[0 + messageSize * b] =	64 + 128;	//Set flag indicating new update (>128)
	}
	//2. Send it: 
	updateMessage(message);
	delete message;	
}

unsigned char _discritizePhase(float phase, int discretePhaseMax) {
	static const float TWO_PI = 2 * ((float)M_PI);
	float modPhase = fmodf(phase, TWO_PI);
	if (modPhase < 0)
		modPhase += TWO_PI;
	float normalizedPhase = modPhase / TWO_PI;

	int discretePhase = (int)(normalizedPhase * discretePhaseMax);
	return (unsigned char)(discretePhase);
}
unsigned char _discritizeAmplitude(float amplitude, int discreteAmplitudeMax) {
	float step = (discreteAmplitudeMax * 2.0 / M_PI) * asin(amplitude);
	return (unsigned char)step;
}
unsigned char* AsierInhoUDPImpl::discretize(float* phases, float* amplitudes, int discretePhaseMax) {
	int discreteAmplitudeMax = discretePhaseMax / 2;

	unsigned char message[512];
	for (int i = 0; i < numTransducersPerBoard; i++) {
		float correctedPhase = phases[i] - (float)(phaseAdjust[i] * M_PI / 180.0f);
		unsigned char discretePhase = _discritizePhase(correctedPhase, discretePhaseMax);
		unsigned char discreteAmplitude = _discritizeAmplitude(amplitudes[i], discreteAmplitudeMax);

		unsigned char shift = (discreteAmplitudeMax - discreteAmplitude) / 2;
		if (discretePhase + shift < discretePhaseMax)
			discretePhase = discretePhase + shift;
		else
			discretePhase = discretePhase + shift - discretePhaseMax;

		int pinIndex = transducerIds[i];
		message[pinIndex] = discretePhase;
		message[pinIndex + numTransducersPerBoard] = discreteAmplitude;
	}
	message[0] += 128;
	return message;
}

/**
	This method disconnects the board, closing COM ports.
*/
void AsierInhoUDPImpl::disconnect() {
	if (status != AsierInhoState::CONNECTED)
		return;
	//0. Notify threads to end and wait for them
	for (int b = 0; b < numBoards; b++) {
		boardWorkerData[b]->running = false;
		pthread_mutex_unlock(&(boardWorkerData[b]->send_signal));
	}
	for (int b = 0; b < numBoards; b++) 
		pthread_join(boardWorkerThreads[b], NULL);
	//1. Destroy threads and related resources (e.g. signals)
	for (int b = 0; b < numBoards; b++) {
		pthread_mutex_destroy(&(boardWorkerData[b]->send_signal));
		pthread_mutex_destroy(&(boardWorkerData[b]->done_signal));
		//FT_Close(boardWorkerData[b]->board);
		//delete boardWorkerData[b]->boardSN;
		delete boardWorkerData[b];
	}
	board.disconnect();
	boardWorkerData.clear();
	boardWorkerThreads.clear();
	status = AsierInhoState::INIT;
}

/**
	This method sends a new divider, which determines the update rate inside the FPGA (= 40000/divider Hz).
*/
void AsierInhoUDPImpl::sendNewDivider(unsigned int newDivider) {
	if (newDivider > 256) {
		AsierInhoUDP::printMessage("The maximum number of divider is 256\n");
		return;
	}
	unsigned char *dividerMessage = new unsigned char[messageSize * numBoards];
	memset(dividerMessage, 0, messageSize * numBoards * sizeof(unsigned char));
	for (int b = 0; b < numBoards; b++) {
		int divider = newDivider;
		dividerMessage[messageSize * b + 0] = 128;
		for (int bit = 0; bit < 8; bit++) {
			dividerMessage[messageSize * b + 26 + bit] = 128 * (divider % 2);
			divider /= 2;
		}
		dividerMessage[messageSize * b + 34] = 128;
	}
	updateMessage(dividerMessage);
	delete[] dividerMessage;
}

/**
	Sends a command to the board to load new phases and amplitudes for its 256 transducers.
*/
void AsierInhoUDPImpl::_sendUpdate(UDPDriver& board, unsigned char* stream, size_t numMessages, size_t messageSize, unsigned int clientID) {
	//4. Send!
	board.sendString(stream, (DWORD)numMessages * messageSize, clientID);
}

