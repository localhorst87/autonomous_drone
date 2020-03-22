#include "DronePhysics.hpp"

using namespace std;

// nature constants --> global. External linkage
extern const float GRAVITY {9.81}; // [m/s²]
extern const float PI {3.141593};

// conversion factors --> internal linkage only
static const float FACTOR_MG2SI {GRAVITY / 1000.0}; // converts the acceleration from the milli-gravity to SI unit [0.001g] --> [m/s²]
static const float FACTOR_DEG2RAD {PI / 180.0}; // converts degree to radiants
static const float FACTOR_CM2M {0.01}; // converts cm to m

MeasurePoint::MeasurePoint(const char* stateString)
{
  this->convertStateString(stateString);
}

ostream& operator<<(ostream& os, const MeasurePoint& mp)
{
  os << "flying time: " << mp.time << endl;
  os << "high to ground: " << mp.getHeightToGround() << " m" << endl;
  os << "speed absolute: " <<  mp.getTotalSpeed() << " m/s" << endl;
  os << "speed vertical: " <<  mp.getVerticalSpeed() << " m/s" << endl;
  os << "acceleration forward: " << mp.getForwardAcceleration() << " m/s²" << endl;
  os << "acceleration vertical: " << mp.getVerticalAcceleration() << " m/s²";
  return os;
}

void MeasurePoint::convertStateString(const char* stateString)
{
  sscanf(stateString, "pitch:%d;roll:%d;yaw:%d;vgx:%d;vgy:%d;vgz:%d;templ:%d;temph:%d;tof:%d;h:%d;bat:%d;baro:%f;time:%d;agx:%f;agy:%f;agz:%f",
  &(MeasurePoint::pose.angleY),
  &(MeasurePoint::pose.angleX),
  &(MeasurePoint::pose.angleZ),
  &(MeasurePoint::pose.speedX),
  &(MeasurePoint::pose.speedY),
  &(MeasurePoint::pose.speedZ),
  &(MeasurePoint::state.tempLow),
  &(MeasurePoint::state.tempHigh),
  &(MeasurePoint::altitude.tofHeight),
  &(MeasurePoint::altitude.relHeight),
  &(MeasurePoint::state.battery),
  &(MeasurePoint::altitude.absHeight),
  &(MeasurePoint::time),
  &(MeasurePoint::pose.accelerationX),
  &(MeasurePoint::pose.accelerationY),
  &(MeasurePoint::pose.accelerationZ) );
}

point3d MeasurePoint::localToGlobalPoint(const point3d& localPoint) const
// transforms local ego coordinates to global coordinates (absolute reference to starting pose)
{
  point3d globalPoint;
  angles3d attitude { this->getAttitudeRadiants() };

  globalPoint.x = cos(attitude.y)*cos(attitude.z) * localPoint.x
  + ( sin(attitude.x)*sin(attitude.y)*cos(attitude.z) - cos(attitude.x)*sin(attitude.z) ) * localPoint.y
  + ( cos(attitude.x)*sin(attitude.y)*cos(attitude.z) + sin(attitude.x)*sin(attitude.z) ) * localPoint.z;

  globalPoint.y = cos(attitude.y)*sin(attitude.z) * localPoint.x
  + ( sin(attitude.x)*sin(attitude.y)*sin(attitude.z) + cos(attitude.x)*cos(attitude.z) ) * localPoint.y
  + ( cos(attitude.x)*sin(attitude.y)*sin(attitude.z) - sin(attitude.x)*cos(attitude.z) ) * localPoint.z;

  globalPoint.z = -sin(attitude.y) * localPoint.x
  + sin(attitude.x)*cos(attitude.y) * localPoint.y
  + cos(attitude.x)*cos(attitude.y) * localPoint.z;

  return globalPoint;
}

point3d MeasurePoint::globalToLocalPoint(const point3d& globalPoint) const
{
  point3d localPoint;
  angles3d attitude { this->getAttitudeRadiants() };

  localPoint.x = cos(attitude.y)*cos(attitude.z) * globalPoint.x
  + cos(attitude.y)*sin(attitude.z) * globalPoint.y
  - sin(attitude.y) * globalPoint.z;

  localPoint.y = ( sin(attitude.x)*sin(attitude.y)*cos(attitude.z) - cos(attitude.x)*sin(attitude.z) ) * globalPoint.x
  + ( sin(attitude.x)*sin(attitude.y)*sin(attitude.z) + cos(attitude.x)*cos(attitude.z) ) * globalPoint.y
  + sin(attitude.x)*cos(attitude.y) * globalPoint.z;

  localPoint.z = ( cos(attitude.x)*sin(attitude.y)*cos(attitude.z) + sin(attitude.x)*sin(attitude.z) ) * globalPoint.x
  + ( cos(attitude.x)*sin(attitude.y)*sin(attitude.z) - sin(attitude.x)*cos(attitude.z) ) * globalPoint.y
  + cos(attitude.x)*cos(attitude.y) * globalPoint.z;

  return localPoint;
}

angles3d MeasurePoint::getAttitudeRadiants() const
// returns the angles around the x,y and z-axis in radiants (roll, pitch, yaw)
{
  angles3d attitude {this->pose.angleX * FACTOR_DEG2RAD, this->pose.angleY * FACTOR_DEG2RAD, this->pose.angleZ * FACTOR_DEG2RAD};
  return attitude;
}

float MeasurePoint::getTotalSpeed() const
{
  return FACTOR_CM2M * sqrt( pow(this->pose.speedX, 2.0) + pow(this->pose.speedY, 2.0) + pow(this->pose.speedZ, 2.0) ); // [m/s]
}

float MeasurePoint::getVerticalSpeed() const
{
  return FACTOR_CM2M * this->pose.speedZ; // [m/s]
}

point3d MeasurePoint::getRawAcceleration() const
// returns the raw acceleration in the drone ego COS. No gravity correction !
{
  point3d accLocal {FACTOR_MG2SI * this->pose.accelerationX, FACTOR_MG2SI * this->pose.accelerationY, FACTOR_MG2SI * this->pose.accelerationZ};
  return accLocal; // [m/s²] No gravity correction!
}

point3d MeasurePoint::getGlobalAcceleration() const
{
  point3d accLocal { this->getRawAcceleration() };
  point3d accGlobal { this->localToGlobalPoint(accLocal) };
  accGlobal.z += GRAVITY; // gravity offset correction
  return accGlobal; // [m/s²]
}

point3d MeasurePoint::getLocalAcceleration() const
  // returns the gravity corrected acceleration in the drone ego COS.
{
  point3d accGlobal { this->getGlobalAcceleration() }; // returns gravity corrected acceleration in global COS
  return this->globalToLocalPoint(accGlobal);
}

float MeasurePoint::getForwardAcceleration() const
// returns the acceleration regarding the yaw-direction of the camera view
{
  point3d accGlobal { this->getGlobalAcceleration() };
  angles3d attitude { this->getAttitudeRadiants() };
  return cos(attitude.z)*accGlobal.x + sin(attitude.z)*accGlobal.y; // [m/s²]
}

float MeasurePoint::getVerticalAcceleration() const
{
  point3d accGlobal { this->getGlobalAcceleration() };
  return accGlobal.z; // [m/s²]
}

float MeasurePoint::getHeightToGround() const
{
  return FACTOR_CM2M * this->altitude.tofHeight; // [m]
}

float MeasurePoint::getHeightToStart() const
{
  return FACTOR_CM2M * this->altitude.relHeight; // [m]
}

int MeasurePoint::getDeviceTemperature() const
{
  return (this->state.tempLow + this->state.tempHigh) / 2; // no roundoff...negligible. Unit [°C]
}

int MeasurePoint::getBattery() const
{
  return this->state.battery; // [%]
}
