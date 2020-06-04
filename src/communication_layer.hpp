/*
provides communication classes to send and receive data with the Tello drone
CommandSocket will be used to send commands to the drone
MeasureSocket will be used to receive measurements from the drone
VideoSocket is used to receive H264 encoded data
*/

#ifndef _COMMUNICATION_LAYER_HPP
#define _COMMUNICATION_LAYER_HPP

#include <string>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctime>
#include <algorithm>
#include <unistd.h>
#include "gateway_layer.hpp"

using namespace std;

class TelloSockets: public CommunicationInterface
{
protected:
  const double BLOCKING_TIME_DEFAULT = 0.5; // [s]
  string ip;
  int port;
  int socketFileDesc = -1;
  bool connected = false;
  sockaddr_in address;
  socklen_t addressLength;
  size_t receiveBufferSize = 0;
  uint8_t* receiveBuffer = nullptr;

protected:
  sockaddr* getAddress() const;
  bool disableDefaultBlocking() const;
  int receiveTimeBlocked(uint8_t*, int, double);
  bool createSocket();
  virtual bool configureAddress() = 0;
  virtual bool configureSocket() = 0;
  virtual bool connectSocket() = 0;

public:
  virtual void open() final;
  virtual void close() final;
  virtual bool isConnected() final;
  virtual int sendData(const uint8_t*) const override = 0;
  virtual int receiveData(double) final;
  virtual int receiveData() final;
  virtual uint8_t* getData() const final;
};

class ClientSocket: public TelloSockets
{
protected:
  virtual bool configureAddress() final;
  virtual bool configureSocket() final;
  virtual bool connectSocket() final;

public:
  virtual int sendData(const uint8_t*) const override = 0;
};

class ServerSocket: public TelloSockets
{
private:
  int reuseAddress = 1;

protected:
  virtual bool configureAddress() final;
  virtual bool configureSocket() final;
  virtual bool connectSocket() final;

public:
  virtual int sendData(const uint8_t*) const override = 0;
};

class CommandSocket: public ClientSocket
{
private:
  static const size_t BUFFER_SIZE = 32;

public:
  CommandSocket();
  ~CommandSocket();
  virtual int sendData(const uint8_t*) const final;
};

class SensorSocket: public ServerSocket
{
private:
  static const size_t BUFFER_SIZE = 256;

public:
  SensorSocket();
  ~SensorSocket();
  virtual int sendData(const uint8_t*) const final;
};

class VideoSocket: public ServerSocket
{
private:
  static const size_t BUFFER_SIZE = 2048;

public:
  VideoSocket();
  ~VideoSocket();
  virtual int sendData(const uint8_t*) const final;
};

#endif
