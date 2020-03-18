#ifndef _DRONEPHYSICS_HPP_
#define _DRONEPHYSICS_HPP_

#include <string>
#include <cmath>
#include <iostream>

using namespace std;

struct point3d
{
  float x;
  float y;
  float z;
};

typedef point3d angles3d;

struct dronePose
{
  int speedX; // longitudinal velocity towards global COS [cm/s]
  int speedY; // lateral velocity towards global COS [cm/s]
  int speedZ; // vertical velocity towards global COS [cm/s]
  int angleX; // roll angle [deg] towards global COS (start pose of drone)
  int angleY; // pitch angle [deg] towards global COS (start pose of drone)
  int angleZ; // yaw angle [deg] towards global COS (start pose of drone)
  float accelerationX; // longitudinal acceleration towards DRONE COS (raw, NO gravity correction) [1e-3 g]
  float accelerationY; // lateral acceleration towards DRONE COS (raw, NO gravity correction) [1e-3 g]
  float accelerationZ; // vertical acceleration towards DRONE COS (raw, NO gravity correction) [1e-3 g]
};

struct droneState
{
  int tempLow; // lowest temperature [°C]
  int tempHigh; // highest temperature [°C]
  int battery; // battery charge [%]
};

struct droneAltitude
{
  int tofHeight; // height above ground [cm]
  int relHeight; // height relative to starting height [cm]
  float absHeight; // absolute height above N.N. [m]
};

class MeasurePoint
{
  private:
    int time;
    dronePose pose;
    droneState state;
    droneAltitude altitude;
    point3d localToGlobalPoint(const point3d&) const;
    point3d globalToLocalPoint(const point3d&) const;
    point3d getRawAcceleration() const;
    angles3d getAttitudeRadiants() const;
    void convertStateString(const char*);

  public:
    MeasurePoint(const char*);
    friend ostream& operator<<(ostream&, const MeasurePoint&);
    float getTotalSpeed() const;
    float getVerticalSpeed() const;
    point3d getGlobalAcceleration() const;
    point3d getLocalAcceleration() const;
    float getForwardAcceleration() const;
    float getVerticalAcceleration() const;
    float getHeightToGround() const;
    float getHeightToStart() const;
    int getDeviceTemperature() const;
    int getBattery() const;
};

#endif
