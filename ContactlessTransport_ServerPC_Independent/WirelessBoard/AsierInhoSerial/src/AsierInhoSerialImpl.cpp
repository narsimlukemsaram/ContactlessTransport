#include <src/AsierInhoSerialImpl.h>
#include <sstream>
#include <fstream>


static char consoleLineBuffer[512];

/**
	Method run by each of the wroker threads
*/
void* worker_BoardUpdater(void* threadData) {
	//we read ou configuration
	struct worker_threadData* data = (struct worker_threadData*)threadData;
	
	while (data->running) {
		//Wait for our signal
		pthread_mutex_lock(&data->send_signal);
		//Send update: 		
		AsierInhoSerialImpl::_sendUpdate(data->board, data->dataStream, data->numMessagesToSend, data->messageSize);
		data->numMessagesToSend = 0;
#ifdef _TIME_PROFILING
		struct timeval curTime;
		data->curUpdate = (data->curUpdate + 1) % data->UPS;
		gettimeofday(&curTime, 0x0);
		data->timestamps[data->curUpdate] = computeTimeElapsed(data->referenceTime, curTime);
#endif
		//Send notification signal
		pthread_mutex_unlock(&data->done_signal);		
	}	
	AsierInhoSerial::printMessage("AsierInho Worker finished!\n");
	return 0;
}


#ifdef _TIME_PROFILING
void AsierInhoSerialImpl::_profileTimes() {
	//Lock all boards:
	for (int b = 0; b < boardWorkerData.size(); b++) {
		pthread_mutex_lock(&(boardWorkerData[b]->send_signal));
	}
	//Write all time profiles: 
	for (int b = 0; b < boardWorkerData.size(); b++) {
		FILE* boardFile;
		fopen_s(&boardFile, boardWorkerData[b]->boardSN, "a");
		for (int u = 0; u < boardWorkerData[b]->UPS; u++) {
			int update = (u + boardWorkerData[b]->curUpdate) % boardWorkerData[b]->UPS;
			fprintf_s(boardFile, "%d, %f\n", update, boardWorkerData[b]->timestamps[update]);
		}
		fclose(boardFile);
	}
	//Unlock all boards:
	for (int b = 0; b < boardWorkerData.size(); b++) {
		pthread_mutex_unlock(&(boardWorkerData[b]->send_signal));
	}
	
}

#endif

AsierInhoSerialImpl::AsierInhoSerialImpl() :AsierInhoSerialBoard(), status(INIT) {
	
}

AsierInhoSerialImpl::~AsierInhoSerialImpl() {
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
bool AsierInhoSerialImpl::connect(int bottomBoardID, int topBoardID, int maxNumMessagesToSend) {
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

bool AsierInhoSerialImpl::connect(int numBoards, int * boardIDs, float* matToWorld , int maxNumMessagesToSend)
{
	BoardConfig boardConfig = ParseBoardConfig::readParameters(std::stringstream("board_mini.pat"));
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
#ifdef _TIME_PROFILING
	struct timeval reference;
	gettimeofday(&reference, 0x0);
#endif
	for (int b = 0; b < numBoards; b++) {
		this->boardIDs.push_back(boardIDs[b]);	
		boardWorkerData.push_back(new struct worker_threadData);
		boardWorkerData[b]->dataStream = &(dataStream[b*messageSize*maxNumMessagesToSend]);
		//readBoardParameters(boardIDs[b], &(matToWorld[16*b]), &(transducerPositions[3*256*b]), &(transducerNormals[3 * 256 * b]), &(transducerIds[256*b]), &(phaseAdjust[256*b]), &(amplitudeAdjust[256*b]), &numDiscreteLevels[b], (boardWorkerData[b]));
		copyBoardParameters(boardConfig, boardIDs[b], &(matToWorld[16 * b]), &(transducerPositions[3 * numTransducersPerBoard * b]), &(transducerNormals[3 * numTransducersPerBoard * b]), &(transducerIds[numTransducersPerBoard * b]), &(phaseAdjust[numTransducersPerBoard * b]), &(amplitudeAdjust[numTransducersPerBoard * b]), &numDiscreteLevels[b], (boardWorkerData[b]));
		if (numDiscreteLevels[b] < this->numDiscreteLevels)
			this->numDiscreteLevels = numDiscreteLevels[b];
#ifdef _TIME_PROFILING		//DEBUG: Add time profiling fields:
		boardWorkerData[b]->referenceTime = reference;
		boardWorkerData[b]->curUpdate = 0;
#endif 	//END DEBUG

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
		bool allConnected=true;
		int boardsConnected;
		for (boardsConnected = 0; boardsConnected < numBoards && allConnected; boardsConnected++) {
			allConnected &= initCOMToolkit((boardWorkerData[boardsConnected]->board), boardWorkerData[boardsConnected]->portNumber);
		}

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
				pthread_create(&boardWorkerThreads[b], &attrs 	, &worker_BoardUpdater, ((void*)boardWorkerData[b]));
			
			}			
			Sleep(100);
			AsierInhoSerial::printMessage("AsierInhoSerial connected succesfully\n");
			return true;
		}
		else {
			//Destroy connections created? (0..boardConnected)
			AsierInhoSerial::printWarning("AsierInhoSerial connection failed\n");
			return false;
		}
	}
	AsierInhoSerial::printWarning("AsierInhoSerial is already connected\n");
	return false;
}

void AsierInhoSerialImpl::readParameters(float * transducerPositions, float* transducerNormals, int * transducerIds, int * phaseAdjust, float * amplitudeAdjust, int * numDiscreteLevels)
{
	memcpy(transducerPositions, this->transducerPositions, numTransducersPerBoard * numBoards * 3 * sizeof(float));
	memcpy(transducerNormals, this->transducerNormals, numTransducersPerBoard * numBoards * 3 * sizeof(float));
	memcpy(transducerIds , this->transducerIds, numTransducersPerBoard *numBoards*sizeof(int));
	memcpy(phaseAdjust , this->phaseAdjust, numTransducersPerBoard *numBoards*sizeof(int));
	memcpy(amplitudeAdjust , this->amplitudeAdjust, numTransducersPerBoard *numBoards*sizeof(float));
	*numDiscreteLevels = this->numDiscreteLevels;
}

void AsierInhoSerialImpl::updateBoardPositions(float * matBoardToWorld4x4)
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

void AsierInhoSerialImpl::readBoardParameters(int boardId, float* matToWorld, float* transducerPositions, float* transducerNormals, int* transducerIds, int* phaseAdjust, float* amplitudeAdjust, int* numDiscreteLevels, worker_threadData* thread_data) {
	BoardConfig boardConfig = ParseBoardConfig::readParameters(boardId);
	thread_data->boardSN = new char[strlen(boardConfig.hardwareID) + 1];
	strcpy_s(thread_data->boardSN, strlen(boardConfig.hardwareID) + 1, boardConfig.hardwareID);
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

void AsierInhoSerialImpl::copyBoardParameters(BoardConfig boardConfig, int boardId, float* matToWorld, float* transducerPositions, float* transducerNormals, int* transducerIds, int* phaseAdjust, float* amplitudeAdjust, int* numDiscreteLevels, worker_threadData* thread_data) {
	thread_data->portNumber = boardId;
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

//
bool AsierInhoSerialImpl::initCOMToolkit(COMToolkit2& fthandle, int portNumber) {
	fthandle.connect(portNumber, baudrate);
	if (fthandle.anyErrors()) {
		sprintf_s<512>(consoleLineBuffer, "Could not open USB port %d\n", portNumber);
		AsierInhoSerial::printWarning(consoleLineBuffer);
		return false;
	}

	AsierInhoSerial::printMessage("USB ports open\n");
	return true;

	//FT_STATUS status;
	//status = FT_OpenEx(serialNumber, FT_OPEN_BY_SERIAL_NUMBER, &fthandle);
	//if (status != FT_OK) {
	//	if (status == FT_INVALID_HANDLE)	sprintf_s<512>(consoleLineBuffer, "Could not open USB port, status not ok (%d - invalid handle...)\n", status);
	//	if (status == FT_DEVICE_NOT_FOUND)	sprintf_s<512>(consoleLineBuffer, "Could not open USB port, open status not ok (%d - device not found...)\n", status);
	//	if (status == FT_DEVICE_NOT_OPENED) sprintf_s<512>(consoleLineBuffer, "Could not open USB port, open status not ok (%d - device not opened...)\n", status);
	//	AsierInhoSerial::printWarning(consoleLineBuffer);
	//	return false;
	//}
	//AsierInhoSerial::printMessage("USB ports open\n");
	//status = FT_ResetDevice(fthandle);
	//if (status != FT_OK) {
	//	sprintf_s<512>(consoleLineBuffer, "Could not reset USB ports. Reset status %d\n", status);
	//	AsierInhoSerial::printWarning(consoleLineBuffer);
	//	return false;
	//}

	//// set the transfer mode as Syncronsou mode
	//if (syncMode) {
	//	UCHAR Mask = 0xFF; // Set data bus to outputs
	//	UCHAR mode_rst = 0;
	//	UCHAR mode_sync = 0x40; // Configure FT2232H into 0x40 Sync FIFO mode
	//	FT_SetBitMode(fthandle, Mask, mode_rst); // reset MPSSE
	//	FT_SetBitMode(fthandle, Mask, mode_sync); // configure FT2232H into Sync FIFO mode
	//}

	//// set the latency timer and USB parameters
	//FT_SetLatencyTimer(fthandle, 2);
	//FT_SetUSBParameters(fthandle, 0x10000, 0x10000);
	//FT_Purge(fthandle, FT_PURGE_RX | FT_PURGE_TX);
	//AsierInhoSerial::printMessage("USB ports setup\n");
	//return true;
}

void AsierInhoSerialImpl::updateMessagePerBoard(unsigned char ** message)
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

void AsierInhoSerialImpl::updateMessage(unsigned char* message) {
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

void AsierInhoSerialImpl::updateMessages(unsigned char* message, int numMessagesToSend) {
	//0. Check status
	if (status != AsierInhoState::CONNECTED)
		return;
	if(numMessagesToSend>maxNumMessagesToSend || numMessagesToSend<1)
		AsierInhoSerial::printWarning("AsierInho: The driver cannot send the requested number of messages. Command Ignored.");
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
void AsierInhoSerialImpl::turnTransducersOff() {
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

void AsierInhoSerialImpl::turnTransducersOn() {
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
unsigned char* AsierInhoSerialImpl::discretize(float* phases, float* amplitudes) {
	static int discretePhaseMax = 128;
	static int discreteAmplitudeMax = discretePhaseMax / 2;

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
void AsierInhoSerialImpl::disconnect() {
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
		boardWorkerData[b]->board.closeConnection();
		delete boardWorkerData[b]->boardSN;
		delete boardWorkerData[b];
	}
	boardWorkerData.clear();
	boardWorkerThreads.clear();
	status = AsierInhoState::INIT;
}


/**
	Sends a command to the board to load new phases and amplitudes for its 256 transducers.
*/
void AsierInhoSerialImpl::_sendUpdate(COMToolkit2& board, unsigned char* stream, size_t numMessages, size_t messageSize) {
	//4. Send!
	board.sendString(stream, (DWORD)numMessages * messageSize);

	//FT_STATUS ftStatus; DWORD dataWritten;
	////4. Send!
	//ftStatus = FT_Write(board, stream, (DWORD)numMessages*AsierInhoSerialImpl::messageSize, &dataWritten);
	//if (ftStatus != FT_OK) {
	//	sprintf_s<512>(consoleLineBuffer, "AsierInhoSerialImpl::_sendUpdate error %d\n", ftStatus);
	//	printWarning(consoleLineBuffer);
	//}
}

