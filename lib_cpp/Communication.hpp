/*
provides a communication API to send and receive data with the Tello drone
CommandSocket will be used to send commands to the drone
MeasureSocket will be used to receive measurements from the drone
VideoSocket is used to receive H264 encoded data

To activate a socket use the following order of commands:
configureAddress --> configureSocket --> connectSocket
*/

#ifndef _COMMUNICATION_HPP_
#define _COMMUNICATION_HPP_

#include <string>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

class TelloSockets
{
  protected:
    string ip;
    int port;
    int socketFileDesc;
    sockaddr_in address;
    socklen_t addressLength;

  protected:
    sockaddr* getAddress();

  public:
    bool createSocket();
    bool disconnectSocket();
    bool disableBlocking();
    virtual bool configureAddress() = 0;
    virtual bool configureSocket() = 0;
    virtual bool connectSocket() = 0;
};

class ClientSocket: public TelloSockets
{
  public:
    virtual bool configureAddress() final;
    virtual bool configureSocket() final;
    virtual bool connectSocket() final;
};

class ServerSocket: public TelloSockets
{
  private:
    int reuseAddress = 1;

  public:
    virtual bool configureAddress() final;
    virtual bool configureSocket() final;
    virtual bool connectSocket() final;
};

class CommandSocket: public ClientSocket
{
  private:
    static const size_t BUFFER_SIZE = 32;
    char responseBuffer[BUFFER_SIZE];

  public:
    CommandSocket();
    ~CommandSocket();
    bool sendCommand(const char*) const;
    char* getResponse();
};

class MeasureSocket: public ServerSocket
{
  private:
    static const size_t BUFFER_SIZE = 256;
    char measurementBuffer[BUFFER_SIZE];

  public:
    MeasureSocket();
    ~MeasureSocket();
    char* getMeasures();
};

class VideoSocket: public ServerSocket
{
  private:
    static const int PACKET_SIZE = 1460;
    static const size_t BUFFER_SIZE = 2048;

  public:
    VideoSocket();
    ~VideoSocket();
    int getRawVideoData(uint8_t*);
};

#endif
