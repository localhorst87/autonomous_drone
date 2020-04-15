#include "Communication.hpp"

using namespace std;

bool TelloSockets::createSocket()
// creates a UDP socket. Returns true/false upon successful/unsuccesfull completion.
{
  this->socketFileDesc = socket(AF_INET, SOCK_DGRAM, 0);
  return this->socketFileDesc != -1;
}

bool TelloSockets::disconnectSocket()
// destroys the socket. Returns true/false upon successful/unsuccesfull shutdown
{
  return close(this->socketFileDesc) == 0;
}

bool TelloSockets::disableDefaultBlocking()
// recv() - operation will no longer infinitely block the command, if no data is received.
// Returns true/false upon successful/unsuccesfull configuration
{
  return fcntl(this->socketFileDesc, F_SETFL, O_NONBLOCK) == 0;
}

sockaddr* TelloSockets::getAddress()
// returns the sockaddr pointer required to establish a connection
{
  return (sockaddr*)&this->address;
}

int TelloSockets::receiveTimeBlocked(uint8_t* buffer, int bufferSize)
// tries to receive byte stream for a specified time defined in BLOCKING_TIME
// property. If bytes are received the function returns immediately the number
// of received bytes and fills the buffer.
// Else it returns 0 if no bytes were received within the blocking time.
{
  int nBytesReceived {0};
  time_t startTime, currentTime;
  double timeElapsed {0};

  time(&startTime);

  while(nBytesReceived <= 0 && timeElapsed < BLOCKING_TIME)
  {
    nBytesReceived = recv(this->socketFileDesc, buffer, bufferSize, 0);

    time(&currentTime);
    timeElapsed = difftime(currentTime, startTime);
  }

  return max(0, nBytesReceived);
}

bool ClientSocket::configureAddress()
// configures the sockaddr_in object required to establish a connection.
// Returns true/false upon successful/unsuccesfull conversion of the address.
{
  this->addressLength = sizeof(this->address);
  this->address.sin_family = AF_INET;
  this->address.sin_port = htons(this->port);

  return inet_pton(AF_INET, this->ip.c_str(), &this->address.sin_addr ) == 1;
}

bool ClientSocket::configureSocket()
// performs configuration steps to be done before establishing the connection.
// Returns true/false upon successful/unsuccesfull configuration
{
  bool blockingDisabled { this->disableDefaultBlocking() };
  bool addressConfigures { this->configureAddress() };

  return blockingDisabled && addressConfigures;
}

bool ClientSocket::connectSocket()
// requests a connection to be made on the socket.
// returns true/false upon successful/unsuccesfull completion
{
  sockaddr* serverAddress { this->getAddress() };
  return connect(this->socketFileDesc, serverAddress, this->addressLength) == 0;
}

bool ServerSocket::configureAddress()
// configures the sockaddr_in object required to establish a connection.
// Returns true/false upon successful/unsuccesfull conversion of the address.
{
  this->address.sin_family = AF_INET;
  this->address.sin_addr.s_addr = htonl(INADDR_ANY);
  this->address.sin_port = htons(this->port);
  this->addressLength = sizeof(this->address);

  return true;
}

bool ServerSocket::configureSocket()
// performs configuration steps to be done before establishing the connection.
// Returns true/false upon successful/unsuccesfull configuration
{
  bool blockingDisabled { this->disableDefaultBlocking() };
  bool configSet { this->configureAddress() };
  bool optionsSet = setsockopt(this->socketFileDesc, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                    &this->reuseAddress, sizeof(int) ) == 0;

  return configSet && optionsSet;
}

bool ServerSocket::connectSocket()
// assigns the configured address to the socket
{
  sockaddr* serverAddress { this->getAddress() };
  bool isBind = bind(this->socketFileDesc, serverAddress, this->addressLength) == 0;

  return isBind;
}

CommandSocket::CommandSocket()
{
  this->ip = "192.168.10.1";
  this->port = 8889;
}

CommandSocket::~CommandSocket()
{
  this->disconnectSocket();
}

bool CommandSocket::sendCommand(const char* command) const
// sends a command to the Tello drone
// returns true/false upon successful/unsuccesfull completion
{
  return send(this->socketFileDesc, command, strlen(command) + 1, 0) != -1; // + 1 to send null termination
}

uint8_t* CommandSocket::getResponse()
// reads and returns the response to the last sent command
{
  memset(this->responseBuffer, 0, this->BUFFER_SIZE);
  this->receiveTimeBlocked(this->responseBuffer, this->BUFFER_SIZE);

  return this->responseBuffer;
}

MeasureSocket::MeasureSocket()
{
  this->ip = "0.0.0.0";
  this->port = 8890;
}

MeasureSocket::~MeasureSocket()
{
  this->disconnectSocket();
}

uint8_t* MeasureSocket::getMeasures()
// reads and returns the drone measurements (in the ego frame)
{
  memset(this->measurementBuffer, 0, this->BUFFER_SIZE);
  this->receiveTimeBlocked(this->measurementBuffer, this->BUFFER_SIZE);

  return this->measurementBuffer;
}

VideoSocket::VideoSocket()
{
  this->ip = "0.0.0.0";
  this->port = 11111;
}

VideoSocket::~VideoSocket()
{
  this->disconnectSocket();
}

int VideoSocket::getRawVideoData(uint8_t* buffer)
// receives encoded video data and stores it in the given buffer
// make sure buffer has allocated BUFFER_SIZE = 2048 Bytes!
{
  int nBytes = this->receiveTimeBlocked(buffer, this->BUFFER_SIZE);

  return nBytes;
}
