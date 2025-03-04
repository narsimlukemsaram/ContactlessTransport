#ifndef _ASIERINHO_SERIAL_IMPLEMENTATION
#define _ASIERINHO_SERIAL_IMPLEMENTATION
#include <AsierInhoSerial_Prerequisites.h>
#include <AsierInhoSerial.h>
#include <src/ParseBoardConfig.h>
#include <math.h> 
#include <stdio.h>
#include <pthread.h>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
//#include "ftd2xx.h"
#include "COMToolkit2.h"
#include <../Helper/TimeFunctions.h>

using namespace AsierInhoSerial;

struct worker_threadData {
	COMToolkit2 board;				//Port to write to
	char* boardSN;
	int portNumber;
	bool running = true;
	pthread_mutex_t send_signal;	//This mutex controls access to our signal. 
	pthread_mutex_t done_signal;	//We send this signal, once we are done updating.
	unsigned char* dataStream;	//We read the phases/amplitudes to send from this buffer.	
	int numMessagesToSend;
	size_t messageSize;
#ifdef _TIME_PROFILING		//DEBUG: Add time profiling fields:	
	static const int UPS = 8000;
	int curUpdate;
	float timestamps[UPS];
	struct timeval referenceTime;
#endif
};

class AsierInhoSerialImpl : public AsierInhoSerialBoard {
	//ATTRIBUTES:
public:
	static const int baudrate = 2000000;
protected:
	std::vector<int> boardIDs;// botBoardID, topBoardID;
	int numBoards;// = 2;
	int numTransducersPerBoard;
	int numDiscreteLevels;
	int maxNumMessagesToSend;
	size_t messageSize;
	//Callibration parameters...
	int* transducerIds;// [256 * numBoards];
	int* phaseAdjust;// [256 * numBoards];
	float* transducerPositions;//[3*256 * numBoards]
	float* transducerNormals;//[3*256 * numBoards]
	float* amplitudeAdjust;
	std::vector<char*> serialNumbers;// bottomSerialNumber, *topSerialNumber;
	enum AsierInhoState { INIT = 0, CONNECTED };
	int status;
	//shared resource pointing towards the mesages that the boards need to send to update the boards.
	unsigned char* dataStream;// [messageSize*numBoards];		//It can fit up to the message to send (phases+amplitudes)
	
	//Worker threads: They wait untill notified to update phases, and send an event back (XXBoardDone_signal) when the update is complete.
	std::vector<pthread_t> boardWorkerThreads;// topBoardThread;
	std::vector<worker_threadData*> boardWorkerData;

public:
	AsierInhoSerialImpl();
	~AsierInhoSerialImpl();
	virtual bool connect(int bottomBoardID, int topBoardID, int maxNumMessagesToSend );
	virtual bool connect(int numBoards, int* boardIDs, float* matBoardToWorld4x4, int maxNumMessagesToSend );
	virtual void readParameters(float* transducerPositions, float* transducerNormals, int* transducerIds, int* phaseAdjust, float* amplitudeAdjust, int* numDiscreteLevels);
	virtual size_t totalTransducers() { return totalBoards() * numTransducersPerBoard; };
	virtual size_t totalBoards() { return numBoards; }
	//virtual void numTransducersPerBoard(size_t* out_TransPerBoard) {
	//	for (int b = 0; b < numBoards; b++)
	//		out_TransPerBoard[b] = 256;
	//}
	virtual void updateBoardPositions(float*matBoardToWorld4x4);	
	virtual void updateMessagePerBoard(unsigned char** message) ;
	virtual void updateMessage(unsigned char* messages);
	virtual void updateMessages(unsigned char* messages, int numMessagesToSend);
	virtual void turnTransducersOn();
	virtual void turnTransducersOff();
	virtual unsigned char* discretize(float* phases, float* amplitudes);
	virtual void disconnect();
	static void _sendUpdate(COMToolkit2& board, unsigned char* stream, size_t numMessages, size_t messageSize);
#ifdef _TIME_PROFILING
	virtual void _profileTimes();
#endif

protected:
	
	void readBoardParameters(int boardId, float* matToWorld, float* transducerPositions, float* transducerNormals, int* transducerIds, int* phaseAdjust, float* amplitudeAdjust, int* numDiscreteLevels, worker_threadData* thread_data);
	void copyBoardParameters(BoardConfig boardConfig, int boardId, float* matToWorld, float* transducerPositions, float* transducerNormals, int* transducerIds, int* phaseAdjust, float* amplitudeAdjust, int* numDiscreteLevels, worker_threadData* thread_data);
	bool initCOMToolkit(COMToolkit2& fthandle, int portNumber);

};

#endif