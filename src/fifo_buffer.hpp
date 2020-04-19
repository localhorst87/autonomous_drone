/*
FifoBuffer is a container adapter class used in the Logging class
No need for further desciption. It uses the FIFO-principle...
*/

#ifndef _FIFOBUFFER_HPP_
#define _FIFOBUFFER_HPP_

#include <deque>
#include <stdexcept>

using namespace std;

template <class T>
class FifoBuffer
{
  private:
    deque<T> elements;
    int maxSize;

  public:
    FifoBuffer();
    FifoBuffer(int);
    int getMaxSize();
    int size();
    bool isEmpty();
    bool push(T&);
    bool push(T&&);
    T& pop();
};

// template definition must go in the same file to avoid linking errors...

template<class T>
FifoBuffer<T>::FifoBuffer() : maxSize(-1) { }

template<class T>
FifoBuffer<T>::FifoBuffer(int maxSize)
{
  if (maxSize >= 0)
    this->maxSize = maxSize;
  else
    this->maxSize = -1;
}

template<class T>
int FifoBuffer<T>::getMaxSize()
{
  return this->maxSize;
}

template<class T>
int FifoBuffer<T>::size()
{
  return this->elements.size();
}

template<class T>
bool FifoBuffer<T>::isEmpty()
{
  return this->elements.empty();
}

template<class T>
bool FifoBuffer<T>::push(T& newElement)
{
  if ( (this->maxSize > 0 && this->size() < this->maxSize) || this->maxSize == -1 )
  {
    this->elements.push_back(newElement);
    return true;
  }
  else
    return false;
}

template<class T>
bool FifoBuffer<T>::push(T&& newElement)
{
  if ( (this->maxSize > 0 && this->size() < this->maxSize) || this->maxSize == -1 )
  {
    this->elements.push_back(newElement);
    return true;
  }
  else
    return false;
}

template<class T>
T& FifoBuffer<T>::pop()
{
  if (this->size() == 0) throw out_of_range("No elements available");

  T& outElement = this->elements.front();
  this->elements.pop_front();

  return outElement;
}

# endif
