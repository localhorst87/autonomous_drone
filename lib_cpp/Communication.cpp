#include "Communication.hpp"

using namespace std;

bool TelloSockets::createSocket()
// creates a UDP socket. Returns true/false upon successful/unsuccesfull completion.
{
  this->socketFileDesc = socket(AF_INET, SOCK_DGRAM, 0);
  return this->socketFileDesc != -1;
}

bool TelloSockets::disconnectSocket()
// shuts down socket send and receive operations.
// Returns true/false upon successful/unsuccesfull shutdown
{
  return shutdown(this->socketFileDesc, SHUT_RDWR) == 0;
}

sockaddr* TelloSockets::getAddress()
// returns the sockaddr pointer required to establish a connection
{
  return (sockaddr*)&this->address;
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
  return this->configureAddress();
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
  bool configSet { this->configureAddress() };
  bool optionsSet = setsockopt(this->socketFileDesc, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &this->reuseAddress, sizeof(int) ) == 0;

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

bool CommandSocket::sendCommand(const char* command) const
// sends a command to the Tello drone
// returns true/false upon successful/unsuccesfull completion
{
  return send(this->socketFileDesc, command, strlen(command) + 1, 0) != -1; // + 1 to send null termination
}

char* CommandSocket::getResponse()
// reads and returns the response to the last sent command
{
  memset(this->responseBuffer, 0, this->BUFFER_SIZE);
  recv(this->socketFileDesc, this->responseBuffer, this->BUFFER_SIZE, 0);

  return this->responseBuffer;
}

MeasureSocket::MeasureSocket()
{
  this->ip = "0.0.0.0";
  this->port = 8890;
}

char* MeasureSocket::getMeasures()
// reads and returns the drone measurements (in the ego frame)
{
  memset(this->measurementBuffer, 0, this->BUFFER_SIZE);
  recv(this->socketFileDesc, this->measurementBuffer, this->BUFFER_SIZE, 0);

  return this->measurementBuffer;
}

VideoSocket::VideoSocket()
{
  this->ip = "0.0.0.0";
  this->port = 11111;
}

int VideoSocket::getRawVideoData(uint8_t* buffer)
// receives encoded video data and stores it in the given buffer
// make sure buffer has allocated BUFFER_SIZE = 2048 Bytes!
{
  int nBytes = recv(this->socketFileDesc, buffer, this->BUFFER_SIZE, 0);

  return nBytes;
}

bool VideoSocket::isEndOfFrame(const int& nBytesReceived)
// checks if the end of the frame is reached, according to the number of bytes sent in the last datagram packet
{
  return nBytesReceived < PACKET_SIZE;
}
