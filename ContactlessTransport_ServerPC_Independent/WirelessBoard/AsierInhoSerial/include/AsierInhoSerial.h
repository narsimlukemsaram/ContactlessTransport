#ifndef _ASIER_INHO_SERIAL
#define _ASIER_INHO_SERIAL
#include <AsierInhoSerial_Prerequisites.h>
/**
This project contains a simple driver to communicate with out Acoustophoretic board (our PAT).
This provides direct control over each transducer�s phase and amplitude, as well as access to
our MATD mode (high speed computation of single levitation traps, with amplitude and illumination control).
The module is encapsulated into a DLL, which you can use through interfaces in either C++ or C. 
This namespace contains a pure virtual interface (AsierInhoBoard), which cleints can use to control the boards.
It also contains an abstract factory, to allow creation of board controllers (AsierInhoBoards)
without exposing implementation details; as well as other utility functions for the DLL to print
notification, warning and error messages. 
*/
namespace AsierInhoSerial {
	class _AsierInhoSerial_Export AsierInhoSerialBoard {
	protected:
		/**
			AsierInho's cannot be created directly, but you should use the method:
				AsierInho::createAsierInho(BoardType boardType, int bottomBoardID, int topBoardID)
			This allows separating interface from implementation and should allow modifications to the DLL without breaking clients.
		*/
		AsierInhoSerialBoard() { ; }

	public:
		/**
			Virtual destructor (required so that derived classes are deleted properly)
		*/
		virtual ~AsierInhoSerialBoard() { ; }

		//PUBLIC METHODS USED BY CLIENTS:

		/**
			Connects to the boards, using the board IDs specified.
			ID corresponds to the index we labeled on each board. The setup assumes a Top-bottom arrangement (separation 23.88cm)
			If the top ID is set to zero (or not provided, as per the default parameter), this allows single board use as for haptic-only applications.
			Returns NULL if connection failed.
		*/
		virtual bool connect(int bottomBoardID, int topBoardID = 0, int maxNumMessagesToSend = 32) = 0;
		/**
			Connects to an arbitrary number of boards, using the board IDs and locations specified.
			ID corresponds to the index we labeled on each board.
			Matrices specify the location of each board (variable placement).			
			Returns NULL if connection failed.
		*/
		virtual bool connect(int numBoards, int* boardIDs, float* matBoardToWorld4x4, int maxNumMessagesToSend = 32) = 0;
		
		/**
			DEBUG VERSION: Reads the parameters required to use the boards (transducer positions, phase corrections, etc). 
			This method allows you to read the information required by GS_PAT to compute solutions, and create the messages to send to the board.			
			
		*/
		virtual void readParameters(float* transducerPositions, float* transducerNormals, int* transducerIds, int* phaseAdjust, float* amplitudeAdjust, int* numDiscreteLevels)=0;
		virtual size_t totalTransducers() = 0;
		virtual size_t totalBoards() = 0;
		//virtual void numTransducersPerBoard(size_t* out_TransPerBoard) = 0;
		//RUN-TIME METHODS: 
		/**
			Update the position of each board dynamically (no need to stop the device). 
			Please note that:
			   - the number of matrices provided in matBoardToWorld must match the number of boards configured
			   - the order of the matrices must match the order of IDs provided (e.g. if you connected to boards 5,7 and 3,
			     the first matrix is assumed to describe position of board ID5; second matrix is for board ID7 and third matrix for ID3).  
		*/
		virtual void updateBoardPositions(float*matBoardToWorld4x4)=0;
		
		/**
			The method sends ONE message to the boards, with each message read from a different array (message[0], message[1], etc...). 
			The client must provide as many messages as number of boards the device is using. 
			This is used when software synchronization is used, as the format in which the solver works does not interleave messages 
			(i.e., it provides 32 update to the top board, followed by 32 for the bottom, instead of one msg for top, one for bottom, another top, another bottom).
			The message should be properly formatted internally (e.g. first byte in each package is set to 128 + phase[0]; PIN ID mappings) and structured as 512 bytes for each board (e.g. 512B for the bottom board update; 512B for the top).
			NOTE: The GPU solver (GS-PAT) directly discretises and formats the messages, so clients using GS-PAT only need to call this method
			(but not other methods to discretise, etc.).
		*/
		virtual void updateMessagePerBoard(unsigned char** message) = 0;
		/**
			The method sends ONE message to the boards. This is retained for backward compatibility, but is very likely to become deprecated (assuming hardware sync works well)
			The message should be properly formatted internally (e.g. first byte in each package is set to 128 + phase[0]; PIN ID mappings) and structured as 512 bytes for each board (e.g. 512B for the bottom board update; 512B for the top).
			NOTE: The GPU solver (GS-PAT) directly discretises and formats the messages, so clients using GS-PAT only need to call this method
			(but not other methods to discretise, etc.).
		*/
		virtual void updateMessage(unsigned char* message) = 0;
		/**
			The method sends SEVERAL message to the boards. This is the current preferred use, as it makes better usage of USB bandwidth (allows higher update rates). Hardware synchronization is assumed.
			The message should be properly formatted internally (e.g. first byte in each package is set to 128 + phase[0]; PIN ID mappings).
			The internal structure should contain each board updated contiguously. 
			That is, if sending N messages, the message should contain N*512 bytes for first (bottom) board; N*512 bytes for 2nd (bottom) board; etc. 
			WARNING: Please note, you cannot send more that the maximum number of messages you configured for the driver (typically 32). 
				The driver will not send anything if you request it to send more than the maximum number (it does not have enough memory allocated).
			NOTE: The GPU solver (GS-PAT) directly discretises and formats the messages, so clients using GS-PAT only need to call this method
			(but not other methods to discretise, etc.).
		*/
		virtual void updateMessages(unsigned char* message, int numMessagesToSend) = 0;
		/**
			This method turns the transducers off (so that the board does not heat up/die misserably)
			The board is still connected, so it can later be used again (e.g. create new traps)
		*/
		virtual void turnTransducersOn() = 0;
		virtual void turnTransducersOff() = 0;
		/**
			This method discritize phases and amplitudes to get final messages
		*/
		virtual unsigned char* discretize(float *phases, float *amplitudes) = 0;

		/**
			This method disconnects the board, closing the ports.
		*/
		virtual void disconnect() = 0;

# ifdef _TIME_PROFILING
		virtual void _profileTimes() = 0;
# endif
		};
	
	_AsierInhoSerial_Export AsierInhoSerialBoard* createAsierInho();

	void printMessage(const char*);
	void printError(const char*);
	void printWarning(const char*);

	_AsierInhoSerial_Export void RegisterPrintFuncs(void(*p_Message)(const char*), void(*p_Warning)(const char*), void(*p_Error)(const char*));
};


#endif
