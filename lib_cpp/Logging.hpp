#ifndef _LOGGING_HPP_
#define _LOGGING_HPP_

#include <ctime>
#include <string>
#include <vector>
#include "FifoBuffer.hpp"
#include "DronePhysics.hpp"

using namespace std;

class Job
{
  private:
    time_t timestamp;
    string command;
    bool sent;
    string response;
    string errorType;
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
    FifoBuffer<Job> jobsDone;
    FifoBuffer<Job> jobsError;
    vector<MeasurePoint> measurements;

  public:
    bool addJobToDo(const string&);
    Job& getNextJob();
    bool logProcessedJob(Job&);
    void addMeasurement(const MeasurePoint&);
};

#endif
