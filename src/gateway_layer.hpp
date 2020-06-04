/*
This layer is responsible for controlling the communication to the drone.
That implies the activiation and deactivation of the drone's communication
interface, receiving and converting video images, receiving and converting
sensor signals, as well as sending controle commands to the drone.

The raw video bytestream data is H264 encoded. I use libav for decoding, that is
a wrapper around ffmpeg. For any information on functionality, see the libav doc:
https://libav.org/documentation/doxygen/master

Hands-on tutorials are provided in the Libav documentation. Another useful
documentation is provided by roxlu.com:
https://www.roxlu.com/2014/039/decoding-h264-and-yuv420p-playback

Make sure to install libav library before! https://libav.org/download
*/

#ifndef _GATEWAY_LAYER_HPP
#define _GATEWAY_LAYER_HPP

extern "C"
{
  #include <libavcodec/avcodec.h>
  #include <libavutil/avutil.h>
  #include <libswscale/swscale.h>
  #include <libavutil/imgutils.h>
}

#include <stdio.h>
#include <iostream>
#include <vector>
#include <tuple>
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>
#include <map>
#include <utility>
#include "use_case_layer.hpp"

class CommunicationInterface
{
public:
  virtual void open() = 0;
  virtual void close() = 0;
  virtual bool isConnected() = 0;
  virtual int sendData(const uint8_t*) const = 0;
  virtual int receiveData(double) = 0; // user defined blocking time
  virtual int receiveData() = 0; // use default blocking time
  virtual uint8_t* getData() const = 0;
};

class AvFrameConverter
{
private:
  const float SCALE_MAX = 10;
  const float SCALE_MIN = 0.1;
  AVPixelFormat destinationFormat;
  AVPixelFormat sourceFormat;
  int sourceWidth;
  int sourceHeight;
  int destinationWidth;
  int destinationHeight;
  SwsContext* swsContext = nullptr;
  AVFrame* destinationFrame;
  uint8_t* buffer = nullptr;

private:
  void setSwsContext();
  int allocateBuffer();
  void setSourceMetaData(const AVFrame&);
  void setDestinationMetaData();
  void scaleFrame(float);

public:
  AvFrameConverter();
  ~AvFrameConverter();
  AVFrame convert(const AVFrame&, const AVPixelFormat&, const float);
};

class ImageConverter
{
private:
  AvFrameConverter avFrameConverter;
  AVPixelFormat avPixelFormat;
  int width;
  int height;
  int nChannels;
  int nPoints;

private:
  void fillMetaData(const AVFrame&, const ImageFormat&);
  int getNumberOfChannels(const ImageFormat&) const;
  ThreeDimByteVector avFrameToImage(const AVFrame&, const PixelIndexOrder&) const;
  ThreeDimByteVector getPixels(const uint8_t*) const;
  ThreeDimByteVector getPlanes(const uint8_t*) const;
  tuple<int, int, int> calcPixelIndices(int, PixelIndexOrder) const;
  AVPixelFormat getAvPixelFormat(const ImageFormat&) const;

public:
  Image convert(AVFrame, const ImageFormat&, const PixelIndexOrder&, float scaleFactor = 1);
};

class VideoDecoder
{
private:
  const AVCodecID CODEC_TYPE = AV_CODEC_ID_H264;
  AVCodecContext* codecContext = nullptr; // info struct about the chosen codec
  AVCodecParserContext* parserContext = nullptr; // info struct regarding the parser (contains the parser itself!)
  AVFrame* frame; // target frame for decoded data
  AVPacket avPacket;

private:
  void configureDecoder();
  bool openCodec();

public:
  VideoDecoder();
  ~VideoDecoder();
  bool decodeParsedData();
  int parseEncodedData(uint8_t*, int);
  bool readyForDecoding() const;
  AVFrame getFrame() const;
};

class Controller
// interface for video and sensor data streaming implementation
{
protected:
  ReceivingBoundary* receivingBoundary;
  CommunicationInterface* comInterface;

protected:
  virtual void transmitNewData() = 0;

public:
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual bool connect() = 0;
  virtual bool disconnect() = 0;
  virtual bool isConnected() = 0;
};

class VideoController : public Controller
{
private:
  const int DATAGRAM_SIZE = 1460;
  ImageFormat targetFormat = RGB_24;
  PixelIndexOrder targetIndexOrder = WIDTH_HEIGHT_CHANNEL;
  VideoDecoder decoder;
  ImageConverter converter;
  Image currentImage;
  bool isNewImage = true;
  bool isProcessing = false;
  uint8_t frameBuffer[32768];
  uint8_t* packetBuffer = nullptr;
  uint8_t* parserBuffer = nullptr;
  thread streamThread;

private:
  void makeImage();
  int collectFrameData();
  bool processFrameData(int);

protected:
  virtual void transmitNewData() final;

public:
  VideoController(ReceivingBoundary*, CommunicationInterface*);
  void setImageFormat(ImageFormat);
  void setPixelIndexOrder(PixelIndexOrder);
  virtual void start() final;
  virtual void stop() final;
  virtual bool connect() final;
  virtual bool disconnect() final;
  virtual bool isConnected() final;
};

class SensorDataConverter
{
private:
  float accelerationX; // longitudinal acceleration towards DRONE COS (raw, NO gravity correction) [1e-3 g]
  float accelerationY; // lateral acceleration towards DRONE COS (raw, NO gravity correction) [1e-3 g]
  float accelerationZ; // vertical acceleration towards DRONE COS (raw, NO gravity correction) [1e-3 g]
  int angleX; // roll angle [deg] towards global COS (start pose of drone)
  int angleY; // pitch angle [deg] towards global COS (start pose of drone)
  int angleZ; // yaw angle [deg] towards global COS (start pose of drone)
  int tofHeight; // height above ground from lidar measurement [cm]
  float absHeight; // absolute height above N.N. from barometer data [m]

private:
  void convertStateString(const char*);
  TranslationPoint getLocalAcceleration() const;
  TranslationPoint getGlobalAcceleration() const;
  RotationPoint getAttitude() const;
  TranslationPoint localToGlobal(const TranslationPoint&, const RotationPoint&) const;
  TranslationPoint globalToLocal(const TranslationPoint&, const RotationPoint&) const;

public:
  SensorDataPoint convert(const char*);
};

class SensorController : public Controller
{
private:
  SensorDataConverter converter;
  SensorDataPoint currentSensorData;
  bool isProcessing = false;
  thread streamThread;

private:
  void makeSensorPointData();

protected:
  virtual void transmitNewData() final;

public:
  SensorController(ReceivingBoundary*, CommunicationInterface*);
  virtual void start() final;
  virtual void stop() final;
  virtual bool connect() final;
  virtual bool disconnect() final;
  virtual bool isConnected() final;
};

class UnitConverter
// used to convert the quantities of TranslationPoints to a target unit
{
private:
  map<pair<string,string>, float> conversionTable;

private:
  float getFactor(string, string);

public:
  UnitConverter();
  void convert(TranslationPoint*, string);
  TranslationPoint convert(TranslationPoint, string);
  void convert(float*, string, string);
  float convert(float, string, string);
};

#endif
