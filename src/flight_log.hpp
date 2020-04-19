/*
contains classes for logging jobs performed or to be performed by the drone
The Job class is a wrapper for drone commands
The Log class is a buffer for jobs and measurements of the drone
*/

#ifndef _FLIGHT_LOG_HPP
#define _FLIGHT_LOG_HPP

#include <ctime>
#include <string>
#include <vector>
#include <deque>
#include <stdexcept>
#include "fifo_buffer.hpp"
#include "measurements.hpp"

using namespace std;

class Job
{
  private:
    time_t timestamp;
    string command;
    bool sent;
    string response;
    string errorType;

  private:
		string getMainCommand() const;

  public:
    Job(const string&);
    Job(const Job&) = default; // copy constructor is used by deque push methods. Leave it default!
    const string& getCommand();
    char* getJobtime() const;
    bool isValidCommand() const;
    bool isExecuting() const;
		bool isDone() const;
		bool finishedSuccessful() const;
};

class Log
{
  private:
    FifoBuffer<Job> jobsToDo;
    vector<MeasurePoint> measurements;

  public:
    bool addJobToDo(const string&);
    Job& getNextJob();
    void addMeasurement(const MeasurePoint&);
};

#endif
