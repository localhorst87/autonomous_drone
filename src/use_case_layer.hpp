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

class ReceivingBoundary
{
public:
  virtual void processImage(Image) = 0;
  // virtual void processSensorData(SensorDataPoint) = 0;
};

class UseCaseInteractor : public ReceivingBoundary
// Dummy for testing the runtime
{
public:
  FifoBuffer<Image> imageBuffer;

public:
  UseCaseInteractor();
  virtual void processImage(Image) final;
};

#endif
