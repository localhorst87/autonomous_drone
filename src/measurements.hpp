#ifndef _MEASUREMENTS_HPP
#define _MEASUREMENTS_HPP

#include <string>
#include <cmath>
#include <iostream>

using namespace std;

struct Point3d
{
  float x;
  float y;
  float z;
};

typedef Point3d AnglesEuler;

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

  private:
    Point3d localToGlobalPoint(const Point3d&) const;
    Point3d globalToLocalPoint(const Point3d&) const;
    Point3d getRawAcceleration() const;
    AnglesEuler getAttitudeRadiants() const;

  public:
    MeasurePoint();
    void convertStateString(const char*);
    friend ostream& operator<<(ostream&, const MeasurePoint&);
    float getTotalSpeed() const;
    float getVerticalSpeed() const;
    Point3d getGlobalAcceleration() const;
    Point3d getLocalAcceleration() const;
    float getForwardAcceleration() const;
    float getVerticalAcceleration() const;
    float getHeightToGround() const;
    float getHeightToStart() const;
    int getDeviceTemperature() const;
    int getBattery() const;
};

#endif
