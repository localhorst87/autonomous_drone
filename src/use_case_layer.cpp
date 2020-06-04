#include "use_case_layer.hpp"

using namespace std;

UseCaseInteractor::UseCaseInteractor()
// Dummy for testing runtime
{
  this->imageBuffer = FifoBuffer<Image>{2};
}

void UseCaseInteractor::processImage(Image newImage)
// Dummy for testing
{
  this->imageBuffer.push(newImage);
}

ostream& operator<<(ostream& os, const Image& img)
{
  os << "image format: " << img.format << endl;
  os << "pixel index order: " << img.indexOrder << endl;
  os << "format: (" << img.width << " x " << img.height << " x " << img.nChannels << ")" << endl;
  os << "total number of points: " << img.nPoints << endl;

  return os;
}
