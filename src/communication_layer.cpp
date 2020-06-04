#include "communication_layer.hpp"

using namespace std;

void TelloSockets::open()
{
  bool isCreated { this->createSocket() };
  bool isConfigured { this->configureSocket() };
  bool isConnected { this->connectSocket() };

  this->connected = isCreated && isConfigured && isConnected;
}

void TelloSockets::close()
// destroys the socket. Returns true/false upon successful/unsuccesfull shutdown
{
  if (::close(this->socketFileDesc) == 0)
  {
    this->socketFileDesc = -1;
    this->connected = false;
  }
}

bool TelloSockets::isConnected()
{
  return this->connected;
}

bool TelloSockets::createSocket()
// creates a UDP socket. Returns true/false upon successful/unsuccesfull completion.
{
  if (this->socketFileDesc == -1)
    this->socketFileDesc = socket(AF_INET, SOCK_DGRAM, 0);

  return this->socketFileDesc != -1;
}

bool TelloSockets::disableDefaultBlocking() const
// recv() - operation will no longer infinitely block the command, if no data is received.
// Returns true/false upon successful/unsuccesfull configuration
{
  return fcntl(this->socketFileDesc, F_SETFL, O_NONBLOCK) == 0;
}

sockaddr* TelloSockets::getAddress() const
// returns the sockaddr pointer required to establish a connection
{
  return (sockaddr*)&this->address;
}

int TelloSockets::receiveData(double blockingTime)
// tries to receive data until the given blocking time is elapsed and
// stores data the receiveBuffer. Returns the number of received bytes
{
  memset(this->receiveBuffer, 0, this->receiveBufferSize);
  return this->receiveTimeBlocked(this->receiveBuffer, this->receiveBufferSize, blockingTime);
}

int TelloSockets::receiveData()
// tries to receive data until the default blocking time is elapsed and
// stores data the receiveBuffer. Returns the number of received bytes
{
  memset(this->receiveBuffer, 0, this->receiveBufferSize);
  return this->receiveTimeBlocked(this->receiveBuffer, this->receiveBufferSize, BLOCKING_TIME_DEFAULT);
}

uint8_t* TelloSockets::getData() const
// returns the address of the receiveBuffer
{
  return this->receiveBuffer;
}

int TelloSockets::receiveTimeBlocked(uint8_t* buffer, int bufferSize, double blockingTime)
// tries to receive byte stream for a specified time defined in blockingTime
// property. If bytes are received the function returns immediately the number
// of received bytes and fills the buffer.
// Else it returns 0 if no bytes were received within the blocking time.
{
  int nBytesReceived {0};
  time_t startTime, currentTime;
  double timeElapsed {0};

  time(&startTime);

  while(nBytesReceived <= 0 && timeElapsed < blockingTime)
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
  this->receiveBufferSize = BUFFER_SIZE;
  this->receiveBuffer = (uint8_t*) malloc(BUFFER_SIZE);
}

CommandSocket::~CommandSocket()
{
  this->close();
  if (this->receiveBuffer)
    free(this->receiveBuffer);
}

int CommandSocket::sendData(const uint8_t* data) const
// sends a command to the Tello drone and returns the number of sent bytes
{
  int nBytesSent = send(this->socketFileDesc, data, strlen((char*)data) + 1, 0); // + 1 to send null termination
  return max(0, nBytesSent); // send returns -1 upon error
}

SensorSocket::SensorSocket()
{
  this->ip = "0.0.0.0";
  this->port = 8890;
  this->receiveBufferSize = BUFFER_SIZE;
  this->receiveBuffer = (uint8_t*) malloc(BUFFER_SIZE);
}

SensorSocket::~SensorSocket()
{
  this->close();
  if (this->receiveBuffer)
    free(this->receiveBuffer);
}

int SensorSocket::sendData(const uint8_t* data) const
// to date: no need of sending data over the SensorSocket
{
  return 0;
}

VideoSocket::VideoSocket()
{
  this->ip = "0.0.0.0";
  this->port = 11111;
  this->receiveBufferSize = BUFFER_SIZE;
  this->receiveBuffer = (uint8_t*) malloc(BUFFER_SIZE);
}

VideoSocket::~VideoSocket()
{
  this->close();
  if (this->receiveBuffer)
    free(this->receiveBuffer);
}

int VideoSocket::sendData(const uint8_t* data) const
// to date: no need of sending data over the VideoSocket
{
  return 0;
}
