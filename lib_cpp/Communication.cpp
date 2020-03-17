#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>
#include "Communication.hpp"

#include <iostream>
#include <cerrno>

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

bool CommandSocket::sendCommand(const char* command)
// sends a command to the Tello drone
// returns true/false upon successful/unsuccesfull completion
{
  return send(this->socketFileDesc, command, strlen(command) + 1, 0) != -1; // + 1 to send null termination
}

char* CommandSocket::getResponse()
// reads and returns the response to the last sent command
{
  memset(this->responseBuffer, 0, this->bufferSize);
  recv(this->socketFileDesc, this->responseBuffer, this->bufferSize, 0);

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
  memset(this->measurementBuffer, 0, this->bufferSize);
  recv(this->socketFileDesc, this->measurementBuffer, this->bufferSize, 0);

  return this->measurementBuffer;
}

VideoSocket::VideoSocket()
{
  this->ip = "0.0.0.0";
  this->port = 11111;
}

unsigned char* VideoSocket::getVideoFrame()
// reads raw video stream packets until the frame is complete. Then returns the address of the frame
{
  bool isFrameComplete = false;
  int nBytesReceived;

  this->resetCurrentFrame();

  while (!isFrameComplete)
  {
    nBytesReceived = this->readPacketData();
    if (nBytesReceived > 0) { this->addPacketData(nBytesReceived); }
    isFrameComplete = this->isEndOfFrame(nBytesReceived);
  }

  return this->currentFrame;
}

int VideoSocket::getFrameSize()
// returns the size of the currentFrame
{
  return this->bytesAdded;
}

void VideoSocket::resetCurrentFrame()
// clears the current frame's content to add packets of a new frame
{
  this->bytesAdded = 0;
  memset(this->currentFrame, 0, this->frameSize);
}

int VideoSocket::readPacketData()
// method to read a datagram packet of the video stream (packet size is 1460)
{
  int nBytes = 0;

  memset(this->videoBuffer, 0, this->bufferSize);
  nBytes = recv(this->socketFileDesc, this->videoBuffer, this->bufferSize, 0);

  return nBytes;
}

void VideoSocket::addPacketData(const int& nBytes)
// concatenates the last received binary data to the current video frame binary data
{
  memcpy(this->currentFrame + this->bytesAdded, this->videoBuffer, nBytes);
  this->bytesAdded += nBytes;

  if ( this->isEndOfFrame(nBytes) )
  { this->currentFrame[this->bytesAdded] = '\0'; } // add termination if frame is complete
}

bool VideoSocket::isEndOfFrame(const int& nBytesReceived)
// checks if the end of the frame is reached, according to the number of bytes sent in the last datagram packet
{
  return nBytesReceived < 1460;
}
