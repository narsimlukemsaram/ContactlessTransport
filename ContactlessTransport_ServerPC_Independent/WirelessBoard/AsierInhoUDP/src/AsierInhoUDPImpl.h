#ifndef _ASIERINHO_UDP_IMPLEMENTATION
#define _ASIERINHO_UDP_IMPLEMENTATION
#include <AsierInhoUDP_Prerequisites.h>
#include <AsierInhoUDP.h>
#include <src/ParseBoardConfig.h>
#include <math.h> 
#include <stdio.h>
#include <pthread.h>
#include <vector>
#include <map>
#define _USE_MATH_DEFINES
#include <math.h>
#include <src/UDPDriver.h>

using namespace AsierInhoUDP;

struct worker_threadData {
	unsigned int clientID;
	bool running = true;
	pthread_mutex_t send_signal;	//This mutex controls access to our signal. 
	pthread_mutex_t done_signal;	//We send this signal, once we are done updating.
	unsigned char* dataStream;	//We read the phases/amplitudes to send from this buffer.	
	int numMessagesToSend;
	size_t messageSize;
	void* thisPointer;
};

class AsierInhoUDPImpl : public AsierInhoUDPBoard {
protected:
	std::vector<int> boardIDs;
	int numBoards;
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
	int socketPort;
	std::map<int, char*> IPs;
	enum AsierInhoState { INIT = 0, CONNECTED };
	int status;
	//shared resource pointing towards the mesages that the boards need to send to update the boards.
	unsigned char* dataStream;// [messageSize*numBoards];		//It can fit up to the message to send (phases+amplitudes)
	
	//Worker threads: They wait untill notified to update phases, and send an event back (XXBoardDone_signal) when the update is complete.
	std::vector<pthread_t> boardWorkerThreads;// topBoardThread;
	std::vector<worker_threadData*> boardWorkerData;

	UDPDriver board;
	pthread_mutex_t boardUsed;

public:
	AsierInhoUDPImpl();
	~AsierInhoUDPImpl();
	virtual bool connect(int bottomBoardID, int topBoardID, int maxNumMessagesToSend );
	virtual bool connect(int numBoards, int* boardIDs, float* matBoardToWorld4x4, int maxNumMessagesToSend );
	virtual void readParameters(float* transducerPositions, float* transducerNormals, int* transducerIds, int* phaseAdjust, float* amplitudeAdjust, int* numDiscreteLevels);
	virtual size_t totalTransducers() { return totalBoards() * numTransducersPerBoard; };
	virtual size_t totalBoards() { return numBoards; }
	virtual void updateBoardPositions(float*matBoardToWorld4x4);	
	virtual void updateMessagePerBoard(unsigned char** message) ;
	virtual void updateMessage(unsigned char* messages);
	virtual void updateMessages(unsigned char* messages, int numMessagesToSend);
	virtual void turnTransducersOn();
	virtual void turnTransducersOff();
	virtual unsigned char* discretize(float* phases, float* amplitudes, int discretePhaseMax = 128);
	virtual void disconnect();
	virtual void sendNewDivider(unsigned int newDivider);
	static void _sendUpdate(UDPDriver& board, unsigned char* stream, size_t numMessages, size_t messageSize, unsigned int clientID);

	friend void* worker_BoardUpdater(void* threadData/*, void* driver*/);
protected:
	
	void readBoardParameters(int boardId, float* matToWorld, float* transducerPositions, float* transducerNormals, int* transducerIds, int* phaseAdjust, float* amplitudeAdjust, int* numDiscreteLevels, worker_threadData* thread_data);
	void copyBoardParameters(BoardConfig boardConfig, int boardId, float* matToWorld, float* transducerPositions, float* transducerNormals, int* transducerIds, int* phaseAdjust, float* amplitudeAdjust, int* numDiscreteLevels, worker_threadData* thread_data);
	bool initUDPDriver();
	void readIPConfig(std::string filename);
};

#endif