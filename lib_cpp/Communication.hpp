#ifndef _COMMUNICATION_HPP_
#define _COMMUNICATION_HPP_

#include <string>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>

using namespace std;

class TelloSockets
{
  protected:
    string ip;
    int port;
    int socketFileDesc;
    sockaddr_in address;
    socklen_t addressLength;
    sockaddr* getAddress();

  public:
    bool createSocket();
    bool disconnectSocket();
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
    static const size_t bufferSize = 32;
    char responseBuffer[bufferSize];

  public:
    CommandSocket();
    bool sendCommand(const char*) const;
    char* getResponse();
};

class MeasureSocket: public ServerSocket
{
  private:
    static const size_t bufferSize = 256;
    char measurementBuffer[bufferSize];

  public:
    MeasureSocket();
    char* getMeasures();
};

class VideoSocket: public ServerSocket
{
  private:
    static const int PACKET_SIZE = 1460;
    static const size_t bufferSize = 2048;
    static const size_t frameSize = 32767;
    unsigned char videoBuffer[bufferSize];
    unsigned char currentFrame[frameSize]; // C++14 conformal solution, alternatively using std::byte in C++17
    int bytesAdded;
    void resetCurrentFrame();
    int readPacketData();
    void addPacketData(const int&);
    bool isEndOfFrame(const int&);

  public:
    VideoSocket();
    unsigned char* getRawFrame();
    int getRawFrameSize() const;
};

#endif
