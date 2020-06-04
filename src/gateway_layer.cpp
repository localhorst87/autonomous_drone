#include "gateway_layer.hpp"

using namespace std;

// nature constants --> global. External linkage
extern const float GRAVITY {9.81}; // [m/s²]
extern const float PI {3.141593};

// commands
static const uint8_t TAKEOFF_COMMAND[] = "takeoff";
static const uint8_t LANDING_COMMAND[] = "land";
static const uint8_t STOP_MOTORS_COMMAND[] = "emergency";
static const uint8_t COMMAND_MODE_MSG[] = "command";
static const uint8_t POSITIVE_FEEDBACK_MSG[] = "ok";
static const uint8_t ACTIVATE_VIDEO_MSG[] = "streamon";
static const uint8_t DEACTIVATE_VIDEO_MSG[] = "streamoff";

VideoController::VideoController(ReceivingBoundary* recvBoundary, CommunicationInterface* videoComInterface)
{
  this->receivingBoundary = recvBoundary;
  this->comInterface = videoComInterface;
}

void VideoController::transmitNewData()
{
  this->receivingBoundary->processImage(this->currentImage);
}

void VideoController::start()
{
  if (!this->isProcessing)
  {
    this->isProcessing = true;
    this->streamThread = thread(&VideoController::makeImage, this);
  }
}

void VideoController::stop()
{
  if (this->isProcessing)
  {
    this->isProcessing = false;
    this->streamThread.join();
  }
}

bool VideoController::connect()
{
  if (!this->isConnected() )
    this->comInterface->open();

  return this->isConnected();
}

bool VideoController::disconnect()
{
  if (this->isConnected() )
    this->comInterface->close();

  return this->comInterface->isConnected() == false;
}

bool VideoController::isConnected()
{
  return this->comInterface->isConnected();
}

void VideoController::setImageFormat(ImageFormat imgFormat)
{
  this->targetFormat = imgFormat;
}

void VideoController::setPixelIndexOrder(PixelIndexOrder indexOrder)
{
  this->targetIndexOrder = indexOrder;
}

void VideoController::makeImage()
// collects and processes frame data. Once a frame is available,
// it is converted into an Image and transmitted via the ReceivingBoundary
{
  int nBytesCollected {0};
  bool isFrameAvailable {false};
  Timepoint captureTime;

  while(this->isProcessing)
  {
    if (this->isNewImage)
    {
      captureTime = HiResClock::now();
      this->isNewImage = false;
    }

    nBytesCollected = this->collectFrameData();
    isFrameAvailable = this->processFrameData(nBytesCollected);

    if (isFrameAvailable)
    {
      AVFrame newFrame = this->decoder.getFrame();
      this->currentImage = this->converter.convert(newFrame, this->targetFormat, this->targetIndexOrder);
      this->currentImage.time = captureTime;
      this->transmitNewData();

      this->isNewImage = true;
    }
  }
}

int VideoController::collectFrameData()
// collects data and adds it to the frameBuffer until the number of received bytes
// is below the standard datagram size. Returns the number of collected bytes
{
  int nBytesCollected {0};
  int nBytesRead {0};

  while (true)
  {
    nBytesRead = this->comInterface->receiveData();
    this->packetBuffer = this->comInterface->getData();
    memcpy(&this->frameBuffer[nBytesCollected], this->packetBuffer, nBytesRead);
    nBytesCollected += nBytesRead;

    if (nBytesRead < DATAGRAM_SIZE) // occurs when last datagram packet of a message was sent
      break;
  }

  this->parserBuffer = &this->frameBuffer[0];

  return nBytesCollected;
}

bool VideoController::processFrameData(int nBytesCollected)
// parses and decodes the collected frame data. Returns true if a frame is
// available or false if frame is not yet available and needs additional data
{
  int nParsedBytes {0};
  bool gotPicture {false};

  while (nBytesCollected > 0)
  {
    nParsedBytes = this->decoder.parseEncodedData(this->parserBuffer, nBytesCollected);

    if ( this->decoder.readyForDecoding() )
      gotPicture = this->decoder.decodeParsedData();

    nBytesCollected -= nParsedBytes;
    this->parserBuffer += nParsedBytes;
  }

  return gotPicture;
}

VideoDecoder::VideoDecoder()
{
  avcodec_register_all(); // Registers all the codecs, parsers and bitstream filters enabled at configuration time
  this->configureDecoder();
}

VideoDecoder::~VideoDecoder()
{
  if(this->parserContext)
  {
    av_parser_close(this->parserContext);
    this->parserContext = nullptr;
  }

  if(this->codecContext)
  {
    avcodec_close(this->codecContext);
    av_free(this->codecContext);
    this->codecContext = nullptr;
  }

  if(this->frame)
  {
    av_free(this->frame);
    this->frame = nullptr;
  }
}

void VideoDecoder::configureDecoder()
// opens the H264 codec, initializes its parser and allocates the target frame
{
  if ( !this->openCodec() )
    throw runtime_error("Error opening the codec!");

  this->parserContext = av_parser_init(CODEC_TYPE);
  if (!this->parserContext)
    throw runtime_error("Error initializing parser!");

  this->frame = av_frame_alloc();
  if (!this->frame)
    throw runtime_error("Error allocating frame!");

  av_init_packet(&this->avPacket);
}

bool VideoDecoder::openCodec()
// returns true/false upon successful/unsucćessful opening of the H264 codec
{
  AVCodec* codec = avcodec_find_decoder(CODEC_TYPE); // returns NULL if codec type not found
  if (!codec)
    return false;

  this->codecContext = avcodec_alloc_context3(codec); // allocates an AVCodecContext and set its fields to default values
  if (codec->capabilities & CODEC_CAP_TRUNCATED)
    this->codecContext->flags |= CODEC_FLAG_TRUNCATED;

  if (avcodec_open2(codecContext, codec, nullptr) >= 0) // returns zero on success, a negative value on error
    return true;
  else
    return false;
}

int VideoDecoder::parseEncodedData(uint8_t* encodedData, int size)
// parses the data and returns true if parsing has finished or false if additional encoded data is required.
// Important: For useful output a key frame must be sent by the drone's camera. This can take up to
// a few seconds after starting to receive video stream data!
{
  int nParsedBytes = av_parser_parse2(this->parserContext, this->codecContext,
                                  &this->avPacket.data, &this->avPacket.size,
                                  encodedData, size, 0, 0, AV_NOPTS_VALUE);

  return nParsedBytes;
}

bool VideoDecoder::decodeParsedData()
// main function to decode parsed data
{
  bool gotPicture {false};

  if (avcodec_send_packet(this->codecContext, &this->avPacket) >= 0)
  {
    if (avcodec_receive_frame(this->codecContext, this->frame) >= 0)
      gotPicture = true;
  }

  return gotPicture;
}

bool VideoDecoder::readyForDecoding() const
// indicates if data can be decoded after parsing
{
  return this->avPacket.size > 0;
}

AVFrame VideoDecoder::getFrame() const
// returns the decoced frame
{
  return *this->frame;
}

AvFrameConverter::AvFrameConverter()
{
  this->destinationFrame = av_frame_alloc();
}

AvFrameConverter::~AvFrameConverter()
{
  if (this->buffer)
    free(this->buffer);
  av_frame_free(&this->destinationFrame);
}

AVFrame AvFrameConverter::convert(const AVFrame& frame, const AVPixelFormat& destinationFormat, const float scaleFactor)
// converts the YUV420 frame to and RgbImage and returns the latter
{
  this->destinationFormat = destinationFormat;
  this->setSourceMetaData(frame);
  this->scaleFrame(scaleFactor);
  this->setDestinationMetaData();
  this->setSwsContext();
  this->allocateBuffer();

  av_image_fill_arrays(this->destinationFrame->data, this->destinationFrame->linesize,
                        this->buffer, this->destinationFormat, this->destinationWidth, this->destinationHeight, 1);

  sws_scale(this->swsContext, frame.data, frame.linesize, 0, this->destinationHeight,
            this->destinationFrame->data, this->destinationFrame->linesize);

  return *this->destinationFrame;
}

void AvFrameConverter::setSourceMetaData(const AVFrame& sourceFrame)
// set width, height and format according to the source frame
{
  this->sourceWidth = sourceFrame.width;
  this->sourceHeight = sourceFrame.height;
  this->sourceFormat = (AVPixelFormat)sourceFrame.format;
}

void AvFrameConverter::scaleFrame(float scaleFactor)
// sets width and height of destination frame according to the given scale factor
// with respect to the original frame.
{
  if (scaleFactor > SCALE_MAX || scaleFactor < SCALE_MIN)
  {
    scaleFactor = min(SCALE_MAX, max(SCALE_MIN, scaleFactor) );
    printf("scale factor of %f is out of range! Adjusted scaleFactor to range of [%f, %f]. \n",
            scaleFactor, SCALE_MIN, SCALE_MAX);
  }

  this->destinationWidth = this->sourceWidth * scaleFactor;
  this->destinationHeight = this->sourceHeight * scaleFactor;
}

void AvFrameConverter::setDestinationMetaData()
// sets width, height and format of the destination frame
{
  this->destinationFrame->width = this->destinationWidth;
  this->destinationFrame->height = this->destinationHeight;
  this->destinationFrame->format = this->destinationFormat;
}

void AvFrameConverter::setSwsContext()
// sets the color conversion and scaling context from the given parameters
{
    this->swsContext = sws_getContext(this->sourceWidth, this->sourceHeight, this->sourceFormat, this->destinationWidth, this->destinationHeight,
                                      this->destinationFormat, SWS_BILINEAR, nullptr, nullptr, nullptr);
}

int AvFrameConverter::allocateBuffer()
// get required buffer size and allocate buffer with this size
{
  int size = av_image_fill_arrays(this->destinationFrame->data, this->destinationFrame->linesize,
                                  nullptr, this->destinationFormat, this->destinationWidth, this->destinationHeight, 1);
  if (this->buffer)
    free(this->buffer);

  this->buffer = (uint8_t*) malloc(size);

  return size;
}

Image ImageConverter::convert(AVFrame frame, const ImageFormat& destinationFormat, const PixelIndexOrder& indexOrder, float scaleFactor)
{
  this->fillMetaData(frame, destinationFormat);
  frame = this->avFrameConverter.convert(frame, this->avPixelFormat, scaleFactor);
  ThreeDimByteVector convertedData = this->avFrameToImage(frame, indexOrder);

  Image convertedImage;
  convertedImage.format = destinationFormat;
  convertedImage.indexOrder = indexOrder;
  convertedImage.data = convertedData;
  convertedImage.nPoints = this->nPoints;
  convertedImage.nChannels = this->nChannels;
  convertedImage.width = this->width;
  convertedImage.height = this->height;

  return convertedImage;
}

void ImageConverter::fillMetaData(const AVFrame& frame, const ImageFormat& destinationFormat)
{
  this->avPixelFormat = this->getAvPixelFormat(destinationFormat);
  this->nChannels = this->getNumberOfChannels(destinationFormat);
  this->width = frame.width;
  this->height = frame.height;
  this->nPoints = this->width * this->height * this->nChannels;
}

ThreeDimByteVector ImageConverter::avFrameToImage(const AVFrame& frame, const PixelIndexOrder& idxOrder) const
{
  if (idxOrder == WIDTH_HEIGHT_CHANNEL)
    { return this->getPixels(frame.data[0]); }
  else if (idxOrder == CHANNEL_WIDTH_HEIGHT)
    { return this->getPlanes(frame.data[0]); }
  else
    { throw invalid_argument("Pixel Order unknown!"); }
}

AVPixelFormat ImageConverter::getAvPixelFormat(const ImageFormat& imageFormat) const
{
  switch(imageFormat)
  {
    case RGB_24:
      return AV_PIX_FMT_RGB24;
    case BGR_24:
      return AV_PIX_FMT_BGR24;
    case GRAY_8:
      return AV_PIX_FMT_GRAY8;
    default:
      throw invalid_argument("Image Format not supported!");
  }
}

int ImageConverter::getNumberOfChannels(const ImageFormat& destinationFormat) const
// linesize of a frame can NOT be used to identify the number of channels as
// the linesize may be larger than the size of usable data – there may be extra
// padding present for performance reasons
// *method to be reviewed --> switch to be avoided
{
  switch(destinationFormat)
  {
    case RGB_24:
      return 3;
    case BGR_24:
      return 3;
    case GRAY_8:
      return 1;
    default:
      throw invalid_argument("Image Format not supported!");
  }
}

ThreeDimByteVector ImageConverter::getPixels(const uint8_t* frameData) const
// returns a three-dimensional vector of size width x height x channels. So each pixel in the picture
// contains an uint8_t - vector with the corresponding channels data inside.
{
  ThreeDimByteVector pixels(this->width, vector<ByteVector>( this->height, ByteVector(this->nChannels) ) );

  for (int i = 0; i < this->nPoints; i++)
  {
    auto pixelIdx = calcPixelIndices(i, WIDTH_HEIGHT_CHANNEL);
    pixels[ get<0>(pixelIdx) ][ get<1>(pixelIdx) ][ get<2>(pixelIdx) ] = frameData[i];
  }

  return pixels;
}

ThreeDimByteVector ImageConverter::getPlanes(const uint8_t* frameData) const
// returns a three-dimensional vector of size channels x width x height. So each channels vector
// contains a plane of a uint8_t - vector of the picture's size (width x height)
{
  ThreeDimByteVector planes(this->nChannels, vector<ByteVector>( this->width, ByteVector(this->height) ) );

  for (int i = 0; i < this->nPoints; i++)
  {
    auto pixelIdx = calcPixelIndices(i, CHANNEL_WIDTH_HEIGHT);
    planes[ get<0>(pixelIdx) ][ get<1>(pixelIdx) ][ get<2>(pixelIdx) ] = frameData[i];
  }

  return planes;
}

tuple<int, int, int> ImageConverter::calcPixelIndices(int dataIdx, PixelIndexOrder idxOrder) const
// returns a tuple of the width-, heigth- and color-index of the pixel,
// as a function of the position inside the raw data array
// 0: width-index, 1: height-index, 2: color-index
{
  int dataLinesize = this->nChannels * this->width;
  int heightIdx {dataIdx / dataLinesize};
  int widthIdx { (dataIdx - heightIdx * dataLinesize) / this->nChannels };
  int channelIdx {dataIdx % this->nChannels};

  if (idxOrder == WIDTH_HEIGHT_CHANNEL)
    return make_tuple(widthIdx, heightIdx, channelIdx);
  else if (idxOrder == CHANNEL_WIDTH_HEIGHT)
    return make_tuple(channelIdx, widthIdx, heightIdx);
  else
    throw invalid_argument("Unknown PixelIndexOrder argument");
}

SensorController::SensorController(ReceivingBoundary* recvBoundary, CommunicationInterface* sensorComInterface)
{
  this->receivingBoundary = recvBoundary;
  this->comInterface = sensorComInterface;
}

void SensorController::transmitNewData()
{
  this->receivingBoundary->processSensorData(this->currentSensorData);
}

void SensorController::start()
{
  if (!this->isProcessing)
  {
    this->isProcessing = true;
    this->streamThread = thread(&SensorController::makeSensorPointData, this);
  }
}

void SensorController::stop()
{
  if (this->isProcessing)
  {
    this->isProcessing = false;
    this->streamThread.join();
  }
}

bool SensorController::connect()
{
  if (!this->isConnected() )
    this->comInterface->open();

  return this->isConnected();
}

bool SensorController::disconnect()
{
  if (this->isConnected() )
    this->comInterface->close();

  return this->comInterface->isConnected() == false;
}

bool SensorController::isConnected()
{
  return this->comInterface->isConnected();
}

void SensorController::makeSensorPointData()
// collects and processes sensor data. Once new sensor data is available,
// it is transmitted via the ReceivingBoundary
{
  uint8_t* dataString {nullptr};
  int nBytesRead;
  Timepoint t;

  while(this->isProcessing)
  {
    t = HiResClock::now();
    nBytesRead = this->comInterface->receiveData();
    if (nBytesRead > 0)
    {
      dataString = this->comInterface->getData();
      this->currentSensorData = this->converter.convert( (char*)dataString);
      this->currentSensorData.time = t;
      this->transmitNewData();
    }
  }
}

SensorDataPoint SensorDataConverter::convert(const char* stateString)
{
  UnitConverter unitConverter;
  this->convertStateString(stateString);

  SensorDataPoint dataPoint;
  dataPoint.accelerationLocal = this->getLocalAcceleration();
  dataPoint.accelerationGlobal = this->getGlobalAcceleration();
  dataPoint.rotationGlobal = this->getAttitude();
  dataPoint.heightToGround = unitConverter.convert(this->tofHeight, "cm", "m");
  dataPoint.heightToNN = this->absHeight;

  return dataPoint;
}

void SensorDataConverter::convertStateString(const char* stateString)
{
  sscanf(stateString, "pitch:%d;roll:%d;yaw:%d;vgx:%*d;vgy:%*d;vgz:%*d;templ:%*d;temph:%*d;tof:%d;h:%*d;bat:%*d;baro:%f;time:%*d;agx:%f;agy:%f;agz:%f",
  &(SensorDataConverter::angleY),
  &(SensorDataConverter::angleX),
  &(SensorDataConverter::angleZ),
  &(SensorDataConverter::tofHeight),
  &(SensorDataConverter::absHeight),
  &(SensorDataConverter::accelerationX),
  &(SensorDataConverter::accelerationY),
  &(SensorDataConverter::accelerationZ) );
}

TranslationPoint SensorDataConverter::getLocalAcceleration() const
{
  UnitConverter unitConverter;
  TranslationPoint accLocal;
  accLocal.x = this->accelerationX;
  accLocal.y = this->accelerationY;
  accLocal.z = this->accelerationZ;
  accLocal.unit = "mg";

  unitConverter.convert(&accLocal, "m/s²");

  return accLocal;
}

TranslationPoint SensorDataConverter::getGlobalAcceleration() const
{
  TranslationPoint accLocal { this->getLocalAcceleration() };
  RotationPoint attitude { this->getAttitude() };
  TranslationPoint accGlobal { this->localToGlobal(accLocal, attitude) };
  accGlobal.z += GRAVITY; // gravity offset correction

  return accGlobal;
}

RotationPoint SensorDataConverter::getAttitude() const
// returns the angles around the x,y and z-axis in radiants (roll, pitch, yaw)
{
  UnitConverter unitConverter;
  RotationPoint attitude;
  attitude.x = this->angleX;
  attitude.y = this->angleY;
  attitude.z = this->angleZ;
  attitude.unit = "deg";

  unitConverter.convert(&attitude, "rad");

  return attitude;
}

TranslationPoint SensorDataConverter::localToGlobal(const TranslationPoint& localPoint, const RotationPoint& attitude) const
// transforms local ego coordinates to global coordinates (absolute reference to starting pose)
// attitude's unit must be in radiants!
{
  TranslationPoint globalPoint;

  globalPoint.x = cos(attitude.y)*cos(attitude.z) * localPoint.x
  + ( sin(attitude.x)*sin(attitude.y)*cos(attitude.z) - cos(attitude.x)*sin(attitude.z) ) * localPoint.y
  + ( cos(attitude.x)*sin(attitude.y)*cos(attitude.z) + sin(attitude.x)*sin(attitude.z) ) * localPoint.z;

  globalPoint.y = cos(attitude.y)*sin(attitude.z) * localPoint.x
  + ( sin(attitude.x)*sin(attitude.y)*sin(attitude.z) + cos(attitude.x)*cos(attitude.z) ) * localPoint.y
  + ( cos(attitude.x)*sin(attitude.y)*sin(attitude.z) - sin(attitude.x)*cos(attitude.z) ) * localPoint.z;

  globalPoint.z = -sin(attitude.y) * localPoint.x
  + sin(attitude.x)*cos(attitude.y) * localPoint.y
  + cos(attitude.x)*cos(attitude.y) * localPoint.z;

  globalPoint.unit = localPoint.unit;

  return globalPoint;
}

TranslationPoint SensorDataConverter::globalToLocal(const TranslationPoint& globalPoint, const RotationPoint& attitude) const
// transforms global coordinates to local ego coordinates of the drone
// attitude's unit must be in radiants!
{
  TranslationPoint localPoint;

  localPoint.x = cos(attitude.y)*cos(attitude.z) * globalPoint.x
  + cos(attitude.y)*sin(attitude.z) * globalPoint.y
  - sin(attitude.y) * globalPoint.z;

  localPoint.y = ( sin(attitude.x)*sin(attitude.y)*cos(attitude.z) - cos(attitude.x)*sin(attitude.z) ) * globalPoint.x
  + ( sin(attitude.x)*sin(attitude.y)*sin(attitude.z) + cos(attitude.x)*cos(attitude.z) ) * globalPoint.y
  + sin(attitude.x)*cos(attitude.y) * globalPoint.z;

  localPoint.z = ( cos(attitude.x)*sin(attitude.y)*cos(attitude.z) + sin(attitude.x)*sin(attitude.z) ) * globalPoint.x
  + ( cos(attitude.x)*sin(attitude.y)*sin(attitude.z) - sin(attitude.x)*cos(attitude.z) ) * globalPoint.y
  + cos(attitude.x)*cos(attitude.y) * globalPoint.z;

  localPoint.unit = globalPoint.unit;

  return localPoint;
}

UnitConverter::UnitConverter()
{
  this->conversionTable[make_pair("g", "m/s²")] = GRAVITY;
  this->conversionTable[make_pair("mg", "m/s²")] = 0.001 * GRAVITY;
  this->conversionTable[make_pair("deg", "rad")] = PI / 180.0;
  this->conversionTable[make_pair("cm", "m")] = 0.01;
  this->conversionTable[make_pair("mm", "m")] = 0.001;
}

float UnitConverter::getFactor(string sourceUnit, string targetUnit)
{
  pair<string, string> conversion = make_pair(sourceUnit, targetUnit);
  return this->conversionTable[conversion];
}

void UnitConverter::convert(TranslationPoint* point, string targetUnit)
{
  float factor = this->getFactor(point->unit, targetUnit);
  float viceVersaFactor = this->getFactor(targetUnit, point->unit);

  if (point->unit == targetUnit || (factor == 0 && viceVersaFactor == 0) )
    return; // source and target unit same or conversion not possible / available

  if (viceVersaFactor != 0)
    *point *= 1/viceVersaFactor;
  else
    *point *= factor;

  point->unit = targetUnit;

  return;
}

TranslationPoint UnitConverter::convert(TranslationPoint point, string targetUnit)
{
  float factor = this->getFactor(point.unit, targetUnit);
  float viceVersaFactor = this->getFactor(targetUnit, point.unit);

  if (point.unit == targetUnit || (factor == 0 && viceVersaFactor == 0) )
    return point; // source and target unit same or conversion not possible / available

  if (viceVersaFactor != 0)
    point *= 1/viceVersaFactor;
  else
    point *= factor;

  point.unit = targetUnit;

  return point;
}

void UnitConverter::convert(float* point, string sourceUnit, string targetUnit)
{
  float factor = this->getFactor(sourceUnit, targetUnit);
  float viceVersaFactor = this->getFactor(targetUnit, sourceUnit);

  if (sourceUnit == targetUnit || (factor == 0 && viceVersaFactor == 0) )
    return; // source and target unit same or conversion not possible / available

  if (viceVersaFactor != 0)
    *point *= 1/viceVersaFactor;
  else
    *point *= factor;

  return;
}

float UnitConverter::convert(float point, string sourceUnit, string targetUnit)
{
  float factor = this->getFactor(sourceUnit, targetUnit);
  float viceVersaFactor = this->getFactor(targetUnit, sourceUnit);

  if (sourceUnit == targetUnit || (factor == 0 && viceVersaFactor == 0) )
    return point; // source and target unit same or conversion not possible / available

  if (viceVersaFactor != 0)
    point *= 1/viceVersaFactor;
  else
    point *= factor;

  return point;
}
