#ifndef _USE_CASE_LAYER_HPP
#define _USE_CASE_LAYER_HPP

#include <string>
#include <iostream>
#include <chrono>
#include <math.h>
#include <vector>
#include "fifo_buffer.hpp"

using namespace std;

typedef chrono::high_resolution_clock HiResClock;
typedef HiResClock::time_point Timepoint;

enum ImageFormat
{
  RGB_24,
  BGR_24,
  GRAY_8
};

enum PixelIndexOrder
{
  WIDTH_HEIGHT_CHANNEL,
  CHANNEL_WIDTH_HEIGHT
};

typedef vector<vector<vector<uint8_t>>> ThreeDimByteVector;
typedef vector<uint8_t> ByteVector;

struct Image
{
  Timepoint time;
  ImageFormat format;
  PixelIndexOrder indexOrder;
  ThreeDimByteVector data;
  int nPoints;
  int nChannels;
  int width;
  int height;
};

ostream& operator<<(ostream&, const Image&);

struct TranslationPoint
{
  float x = 0;
  float y = 0;
  float z = 0;
  string unit;
  TranslationPoint& operator+=(const TranslationPoint&);
  TranslationPoint& operator-=(const TranslationPoint&);
  TranslationPoint& operator*=(const float&);
};

bool operator==(const TranslationPoint&, const TranslationPoint&);
bool operator==(const TranslationPoint&, const float&);
bool operator==(const float&, const TranslationPoint&);
bool operator!=(const TranslationPoint&, const TranslationPoint&);
bool operator!=(const TranslationPoint&, const float&);
bool operator!=(const float&, const TranslationPoint&);
TranslationPoint operator+(const TranslationPoint&, const TranslationPoint&);
TranslationPoint operator*(const TranslationPoint&, const float);
TranslationPoint operator*(const float, const TranslationPoint&);

typedef TranslationPoint RotationPoint;

struct SensorDataPoint
{
  Timepoint time;
  TranslationPoint accelerationLocal; // [m/s²]
  TranslationPoint accelerationGlobal; // [m/s²]
  RotationPoint rotationGlobal; // [rad]
  float heightToGround; // [m]
  float heightToNN; // [m]
};

ostream& operator<<(ostream&, const SensorDataPoint&);

struct Job
{
  string command;
  bool sent = false;
  string response;
};

struct ClosedLoopTranslation
{
  TranslationPoint movement;
  bool done = false;
};

struct ClosedLoopRotation
{
  RotationPoint rotation;
  bool done = false;
};

struct OpenLoopMotion
// all members are velocities given in relative percentages from -100 to +100
{
  int x;
  int y;
  int z;
  int yaw;
  bool done = false;
};

class ReceivingBoundary
{
public:
  virtual void processImage(Image) = 0;
  virtual void processSensorData(SensorDataPoint) = 0;
};

class SendingBoundary
{
public:
  virtual void move(ClosedLoopTranslation&) = 0;
  virtual void move(ClosedLoopRotation&) = 0;
  virtual void move(OpenLoopMotion&) = 0;
  virtual bool isExecuting() = 0;
};

class UseCaseInteractor : public ReceivingBoundary
// Dummy for testing the runtime
{
private:
  SendingBoundary* sendingBoundary;
  FifoBuffer<Image> imageBuffer;
  FifoBuffer<SensorDataPoint> sensorDataBuffer;

public:
  UseCaseInteractor(SendingBoundary*);
  virtual void processImage(Image) final;
  virtual void processSensorData(SensorDataPoint) final;
};

#endif
