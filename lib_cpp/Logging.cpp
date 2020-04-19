#include "Logging.hpp"

using namespace std;

const string VALID_COMMANDS[15] = {"takeoff", "land", "emergency", "up", "down", "left", "right", "forward", "back", "cw", "ccw", "speed", "go", "rc", "curve"};

Job::Job(const string& jobCommand) : command(jobCommand), sent(false)
{ time(&this->timestamp); }

const string& Job::getCommand()
// returns the full-length command with main command and arguments
{
	return this->command;
}

string Job::getMainCommand() const
// returns the main command of the job command without any given arguments. For example the main command of "up 170" is "up"
{
	int firstSpacePos { this->command.find(" ") };

	if (firstSpacePos == -1) // no spaces means no arguments
		return this->command;
	else
		return this->command.substr(0, firstSpacePos); // cuts the string after the first space
}

char* Job::getJobtime() const
{
	return ctime(&this->timestamp);
}

bool Job::isValidCommand() const
// checks if the main command (without arguments) is valid
{
	string mainCommand { this->getMainCommand() };
	bool isValid {false};

	for (const string& validCommand : VALID_COMMANDS)
	{
		if (mainCommand == validCommand)
			{
				isValid = true;
				break;
			}
	}

	return isValid;
}

bool Job::isExecuting() const
// checks if the Job is executing at the moment
{
	return this->sent && this->response.length() == 0;
}

bool Job::isDone() const
// checks if the Job has been tried to execute, regardless if successfully or not
{
	return this->response.length() > 0 || this->errorType.length() > 0;
}

bool Job::finishedSuccessful() const
// checks if the the job has been executed successfully
{
	return this->response.length() > 0 && this->errorType.length() == 0;
}

bool Log::addJobToDo(const string& command)
// creates a new Job with the given command and adds it to the FIFO-buffer, if command is valid. Returns true if successful, false if not.
{
  Job newJob {command};

  if ( newJob.isValidCommand() )
  {
		this->jobsToDo.push(newJob);
		return true;
  }
  else
		return false;
}

Job& Log::getNextJob()
// returns the oldest Job to be done (FIFO principle). Throws an out_of_range exception if no jobs are available
{
  if ( this->jobsToDo.isEmpty() )
		throw out_of_range("No Job available");

  return this->jobsToDo.pop(); // a reference to the oldest job of the deque (FIFO principle)
}

void Log::addMeasurement(const MeasurePoint& point)
{
	this->measurements.push_back(point);
}
