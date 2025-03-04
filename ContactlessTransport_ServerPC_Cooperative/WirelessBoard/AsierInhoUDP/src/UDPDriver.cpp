#include <src/UDPDriver.h>

UDPDriver::UDPDriver()
    : status(INIT) {
};

void UDPDriver::initialise(char* serverIP, int socketPort) {
    this->serverIP = serverIP;
    memcpy(this->broadCastIP, serverIP, 14*sizeof(char));
    broadCastIP[10] = '2'; broadCastIP[11] = '5'; broadCastIP[12] = '5';
    this->socketPort = socketPort;
    bool initialized = begin();
    bool isServerBound = bindServer();
    if (initialized && isServerBound)
        status = READY;
}

unsigned int UDPDriver::addClient(char* clientIP) {
    clientIPs.push_back(clientIP);
    return clientIPs.size() - 1;
}

void UDPDriver::sendString(unsigned char* buffer, int numBytes, int clientID) {
    if (status != READY || clientID >= (int)clientIPs.size()) {
        std::cout << "The communication has not been correctly initialised..." << std::endl;
        return; //Either the connection has not been initialized or the client IP address has not been set yet...
    }

    char* clientIDsent;
    if (clientID >= 0) // send to a specific client
        clientIDsent = clientIPs[clientID];
    else { // broad cast to all the possible clients
        clientIDsent = broadCastIP;
    }
    //consider throwing this consumer into a seperate thread
    inet_pton(AF_INET, clientIDsent, &(serverAddr.sin_addr));
    //std::cout << "Sending UDP " << randomString + " packet to : " << ipAddress   << std::endl;
    sendto(serverSocket, (const char*)buffer, numBytes, 0, (const sockaddr*)&serverAddr, sizeof(sockaddr_in));//send the UDP packet
}
void UDPDriver::sendByte(unsigned char byte, int clientID) {
    unsigned char buffer[1] = { byte };
    sendString(buffer, 1, clientID);
}

bool UDPDriver::anyErrors() {
    if (status == ERRORS) 
        return true;
    else 
        return false;
}

void UDPDriver::disconnect() {
    cleanup();
    status = ENDED;
}



/*
initializes Winsock, starts the broadcasting thread, and then enters a loop where it waits for user input to either change the broadcast frequency or exit the program.
*/
bool UDPDriver::begin() {
    // Initialize winsock
    WSADATA wsData;
    WORD ver = MAKEWORD(2, 2);
    int wsOk = WSAStartup(ver, &wsData); //responsible for retrieving details of the winsock implementation that got linked into the executable
    // Check for winsock initialization failure
    if (wsOk != 0) {
        std::cerr << "Can't initialize winsock! Quitting program" << std::endl;
        return false;
    }
    else 
        return true;
}

/*
 Broadcasting happens here in seperate thread.
 It continually sends UDP packets to all IP addresses in the subnet until `whileFlag` is set to `true`.
 The frequency of the broadcasting is controlled by `frequencyDelay` (1/n).
*/
bool UDPDriver::bindServer() {
    serverSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); // scope of whole thread
    if (serverSocket == INVALID_SOCKET) {
        std::cerr << "Can't create a socket! Quitting program" << std::endl;
        closesocket(serverSocket);
        return false;
    }

    serverAddr.sin_family = AF_INET; // ipv4 family
    serverAddr.sin_port = htons(socketPort); //take host byte order (big endian) and return 16 bit network byte; ip port host to network byte                                                              

    inet_pton(AF_INET, serverIP, &(serverAddr.sin_addr));

    //bind ip addresses once at the start of this loop, only send data to the bound ip addresses
    if (bind(serverSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cout << "Server bind failed with " << serverIP << "; Error: " << (int)WSAGetLastError() << std::endl;
        return false;
    }
    else {
        std::cout << "Bound Server: " << serverIP << std::endl;
    }

    Sleep(500);
    //closesocket(serverSocket);
    return true;
}

/*
Clean up winsock resources
*/
void UDPDriver::cleanup() {
    WSACleanup();
}
