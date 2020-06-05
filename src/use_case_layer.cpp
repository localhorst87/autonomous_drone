#include "use_case_layer.hpp"

using namespace std;

UseCaseInteractor::UseCaseInteractor(SendingBoundary* motionExecuter) :
sendingBoundary(motionExecuter)
{
  // dummy for testing purposes
  this->imageBuffer = FifoBuffer<Image>{10};
  this->sensorDataBuffer = FifoBuffer<SensorDataPoint>{100};
}

void UseCaseInteractor::processImage(Image newImage)
// Dummy for testing
{
  this->imageBuffer.push(newImage);
}

void UseCaseInteractor::processSensorData(SensorDataPoint newData)
// Dummy for testing
{
  this->sensorDataBuffer.push(newData);
}

ostream& operator<<(ostream& os, const Image& img)
{
  os << "image format: " << img.format << endl;
  os << "pixel index order: " << img.indexOrder << endl;
  os << "format: (" << img.width << " x " << img.height << " x " << img.nChannels << ")" << endl;
  os << "total number of points: " << img.nPoints << endl;

  return os;
}

ostream& operator<<(ostream& os, const SensorDataPoint& p)
{
  os << "acceleration local: " <<
    "(x: " << p.accelerationLocal.x <<
    ", y: " << p.accelerationLocal.y  <<
    ", z: " << p.accelerationLocal.z << ") " << p.accelerationLocal.unit << endl;
  os << "acceleration global: " <<
    "(x: " << p.accelerationGlobal.x <<
    ", y: " << p.accelerationGlobal.y  <<
    ", z: " << p.accelerationGlobal.z << ") " << p.accelerationGlobal.unit << endl;
  os << "rotation: " <<
    "(roll: " << p.rotationGlobal.x <<
    ", pitch: " << p.rotationGlobal.y <<
    ", yaw: " << p.rotationGlobal.z << ") " << p.rotationGlobal.unit << endl;
  os << "height to ground: " << p.heightToGround << " m" << endl;
  os << "height over NN: " << p.heightToNN << " m" << endl;

  return os;
}

bool operator==(const TranslationPoint& lhs, const TranslationPoint& rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool operator==(const TranslationPoint& lhs, const float& rhs)
{
  return sqrt( pow(lhs.x, 2) + pow(lhs.y, 2) + pow(lhs.z, 2) ) == rhs;
}

bool operator==(const float& lhs, const TranslationPoint& rhs)
{
  return sqrt( pow(rhs.x, 2) + pow(rhs.y, 2) + pow(rhs.z, 2) ) == lhs;
}

bool operator!=(const TranslationPoint& lhs, const TranslationPoint& rhs)
{
  return lhs.x != rhs.x || lhs.y != rhs.y || lhs.z != rhs.z;
}

bool operator!=(const TranslationPoint& lhs, const float& rhs)
{
  return sqrt( pow(lhs.x, 2) + pow(lhs.y, 2) + pow(lhs.z, 2) ) != rhs;
}

bool operator!=(const float& lhs, const TranslationPoint& rhs)
{
  return sqrt( pow(rhs.x, 2) + pow(rhs.y, 2) + pow(rhs.z, 2) ) != lhs;
}

TranslationPoint operator+(const TranslationPoint& lhs, const TranslationPoint& rhs)
{
  TranslationPoint sum;
  sum.x = lhs.x + rhs.x;
  sum.y = lhs.y + rhs.y;
  sum.z = lhs.z + rhs.z;

  return sum;
}

TranslationPoint& TranslationPoint::operator+=(const TranslationPoint& rhs)
{
  this->x = this->x + rhs.x;
  this->y = this->y + rhs.y;
  this->z = this->z + rhs.z;

  return *this;
}

TranslationPoint operator-(const TranslationPoint& lhs, const TranslationPoint& rhs)
{
  TranslationPoint diff;
  diff.x = lhs.x - rhs.x;
  diff.y = lhs.y - rhs.y;
  diff.z = lhs.z - rhs.z;

  return diff;
}

TranslationPoint& TranslationPoint::operator-=(const TranslationPoint& rhs)
{
  this->x = this->x - rhs.x;
  this->y = this->y - rhs.y;
  this->z = this->z - rhs.z;

  return *this;
}

TranslationPoint operator*(const TranslationPoint& lhs, const float rhs)
{
  TranslationPoint product;
  product.x = lhs.x * rhs;
  product.y = lhs.y * rhs;
  product.z = lhs.z * rhs;

  return product;
}

TranslationPoint operator*(const float lhs, const TranslationPoint& rhs)
{
  TranslationPoint product;
  product.x = rhs.x * lhs;
  product.y = rhs.y * lhs;
  product.z = rhs.z * lhs;

  return product;
}

TranslationPoint& TranslationPoint::operator*=(const float& rhs)
{
  this->x = this->x * rhs;
  this->y = this->y * rhs;
  this->z = this->z * rhs;

  return *this;
}
