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
static const uint8_t ACTIVATE_VIDEO_MSG[] = "streamon";
static const uint8_t DEACTIVATE_VIDEO_MSG[] = "streamoff";
static const string POSITIVE_FEEDBACK_MSG = "ok";

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
  av_log_set_level(AV_LOG_QUIET);
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

CouplingState::~CouplingState()
{ }

void CouplingState::setContext(ControllerProxy* context)
{
  this->controller = context;
}

void DisconnectedState::connect()
{
  CommunicationInterface* cmdInterface = this->controller->getCommandInterface();

  if (!cmdInterface->isConnected() )
    cmdInterface->open();

  if (cmdInterface->isConnected() )
    this->controller->changeCouplingState(new IdleState);
}

void DisconnectedState::disconnect()
{
  // do nothing (already disconnected)
}

void DisconnectedState::activateInteraction()
{
  // do nothing (needs to connect first)
}

void DisconnectedState::activateVideo()
{
  // do nothing (needs interactive mode first)
}

void DisconnectedState::deactivateVideo()
{
  // do nothing (needs interactive vision mode first)
}

void DisconnectedState::getUserInfo()
{
  printf("CoulingState: Disconnected \n");
}

void IdleState::connect()
{
  // do nothing (already connected)
}

void IdleState::disconnect()
{
  CommunicationInterface* cmdInterface = this->controller->getCommandInterface();

  if (cmdInterface->isConnected() )
    cmdInterface->close();

  if (!cmdInterface->isConnected() )
    this->controller->changeCouplingState(new DisconnectedState);
}

void IdleState::activateInteraction()
{
  CommunicationInterface* cmdInterface = this->controller->getCommandInterface();

  if (cmdInterface->sendData(COMMAND_MODE_MSG) == 0) // activate command mode
  {
    cmdInterface->close(); // if sending was not successful, we are not connected
    this->controller->changeCouplingState(new DisconnectedState);
    return;
  }

  if (cmdInterface->receiveData() == 0) // check if got response
  {
    cmdInterface->close(); // if no data received, something must be wrong
    this->controller->changeCouplingState(new DisconnectedState);
    return;
  }

  string response = (char*)cmdInterface->getData(); // get response
  if (response == POSITIVE_FEEDBACK_MSG)
    this->controller->changeCouplingState(new InteractiveState);
}

void IdleState::activateVideo()
{
  // do nothing (needs interactive mode first)
}

void IdleState::deactivateVideo()
{
  // do nothing (needs interactive vision mode first)
}

void IdleState::getUserInfo()
{
  printf("CoulingState: Idle \n");
}

void InteractiveState::connect()
{
  // do nothing (already connected)
}

void InteractiveState::disconnect()
{
  CommunicationInterface* cmdInterface = this->controller->getCommandInterface();

  if (cmdInterface->isConnected() )
    cmdInterface->close();

  if (!cmdInterface->isConnected() )
    this->controller->changeCouplingState(new DisconnectedState);
}

void InteractiveState::activateInteraction()
{
  // do nothing (already in interactive mode)
}

void InteractiveState::activateVideo()
{
  CommunicationInterface* cmdInterface = this->controller->getCommandInterface();

  if (cmdInterface->sendData(ACTIVATE_VIDEO_MSG) == 0) // activate command mode
  {
    cmdInterface->close(); // if sending was not successful, we are not connected
    this->controller->changeCouplingState(new DisconnectedState);
    return;
  }

  if (cmdInterface->receiveData() == 0) // check if got response
  {
    cmdInterface->close(); // if no data received, something must be wrong
    this->controller->changeCouplingState(new DisconnectedState);
    return;
  }

  string response = (char*)cmdInterface->getData(); // get response
  if (response == POSITIVE_FEEDBACK_MSG)
    this->controller->changeCouplingState(new InteractiveVisionState);
}

void InteractiveState::deactivateVideo()
{
  // do nothing (needs interactive vision mode first)
}

void InteractiveState::getUserInfo()
{
  printf("CoulingState: Interactive \n");
}

void InteractiveVisionState::connect()
{
  // do nothing (already connected)
}

void InteractiveVisionState::disconnect()
{
  CommunicationInterface* cmdInterface = this->controller->getCommandInterface();

  if (cmdInterface->isConnected() )
    cmdInterface->close();

  if (!cmdInterface->isConnected() )
    this->controller->changeCouplingState(new DisconnectedState);
}

void InteractiveVisionState::activateInteraction()
{
  // do nothing (already in interactive mode)
}

void InteractiveVisionState::activateVideo()
{
  // do nothing (is already streaming)
}

void InteractiveVisionState::deactivateVideo()
{
  CommunicationInterface* cmdInterface = this->controller->getCommandInterface();

  if (cmdInterface->sendData(DEACTIVATE_VIDEO_MSG) == 0) // activate command mode
  {
    cmdInterface->close(); // if sending was not successful, we are not connected
    this->controller->changeCouplingState(new DisconnectedState);
    return;
  }

  if (cmdInterface->receiveData() == 0) // check if got response
  {
    cmdInterface->close(); // if no data received, something must be wrong
    this->controller->changeCouplingState(new DisconnectedState);
    return;
  }

  string response = (char*)cmdInterface->getData(); // get response
  if (response == POSITIVE_FEEDBACK_MSG)
    this->controller->changeCouplingState(new InteractiveState);
}

void InteractiveVisionState::getUserInfo()
{
  printf("CoulingState: Interactive Vision \n");
}

FlightState::~FlightState()
{ }

void FlightState::setContext(ControllerProxy* context)
{
  this->controller = context;
}

void GroundState::takeoff()
{
  CommunicationInterface* cmdInterface = this->controller->getCommandInterface();

  if (cmdInterface->sendData(TAKEOFF_COMMAND) == 0) // send takeoff command
    return;

  if (cmdInterface->receiveData(BLOCKING_TIME_RECEIVE) == 0) // check if got response
    return;

  string response = (char*)cmdInterface->getData(); // get response
  if (response == POSITIVE_FEEDBACK_MSG)
    this->controller->changeFlightState(new AirState);
}

void GroundState::land()
{
  // do nothing (is already on ground)
}

void GroundState::stopMotors()
{
  // do nothing (motors not running)
}

void GroundState::getUserInfo()
{
  printf("FlightState: Ground \n");
}

void AirState::takeoff()
{
  // do nothing (is already in the air)
}

void AirState::land()
{
  CommunicationInterface* cmdInterface = this->controller->getCommandInterface();

  if (cmdInterface->sendData(LANDING_COMMAND) == 0) // send takeoff command
    return;

  if (cmdInterface->receiveData(BLOCKING_TIME_RECEIVE) == 0) // check if got response
    return;

  string response = (char*)cmdInterface->getData(); // get response
  if (response == POSITIVE_FEEDBACK_MSG)
    this->controller->changeFlightState(new GroundState);
}

void AirState::stopMotors()
{
  CommunicationInterface* cmdInterface = this->controller->getCommandInterface();

  if (cmdInterface->sendData(STOP_MOTORS_COMMAND) == 0) // send takeoff command
    return;

  if (cmdInterface->receiveData() == 0) // check if got response
    return;

  string response = (char*)cmdInterface->getData(); // get response
  if (response == POSITIVE_FEEDBACK_MSG)
    this->controller->changeFlightState(new CrashedState);
}

void AirState::getUserInfo()
{
  printf("FlightState: Air \n");
}

void CrashedState::takeoff()
{
  CommunicationInterface* cmdInterface = this->controller->getCommandInterface();

  if (cmdInterface->sendData(TAKEOFF_COMMAND) == 0) // send takeoff command
    return;

  if (cmdInterface->receiveData(BLOCKING_TIME_RECEIVE) == 0) // check if got response
    return;

  string response = (char*)cmdInterface->getData(); // get response
  if (response == POSITIVE_FEEDBACK_MSG)
    this->controller->changeFlightState(new AirState);
}

void CrashedState::land()
{
  // do nothing (has crashed)
}

void CrashedState::stopMotors()
{
  // do nothing (motors not running)
}

void CrashedState::getUserInfo()
{
  printf("FlightState: Crashed \n");
}

ControllerProxy::ControllerProxy(CommunicationInterface* cmdInterface) :
commandInterface(cmdInterface),
couplingState(nullptr),
flightState(nullptr)
{
  this->changeCouplingState(new DisconnectedState);
  this->changeFlightState(new GroundState);
}

ControllerProxy::~ControllerProxy()
{
  this->stopFlight();
  this->stopVideoStream();
  this->stopSensorStream();
  this->couplingState->disconnect();

  if(this->couplingState != nullptr)
    delete this->couplingState;

  if(this->flightState != nullptr)
    delete this->flightState;
}

void ControllerProxy::setCommandInterface(CommunicationInterface* cmdInterface)
{
  this->commandInterface = cmdInterface;
}

CommunicationInterface* ControllerProxy::getCommandInterface()
{
  return this->commandInterface;
}

void ControllerProxy::changeCouplingState(CouplingState* newCouplingState)
{
  if(this->couplingState != nullptr)
    delete this->couplingState;
  this->couplingState = newCouplingState;
  this->couplingState->setContext(this);
  this->couplingState->getUserInfo();
}

void ControllerProxy::changeFlightState(FlightState* newFlightState)
{
  if(this->flightState != nullptr)
    delete this->flightState;
  this->flightState = newFlightState;
  this->flightState->setContext(this);
  this->flightState->getUserInfo();
}

void ControllerProxy::setVideoController(Controller* vidController)
{
  this->videoController = vidController;
}

void ControllerProxy::setSensorController(Controller* sensController)
{
  this->sensorController = sensController;
}

void ControllerProxy::startFlight()
{
  this->couplingState->connect(); // make sure command interface is connected
  this->couplingState->activateInteraction(); // make sure we are in interactive mode
  this->flightState->takeoff();
}

void ControllerProxy::stopFlight()
{
  this->flightState->land();
}

void ControllerProxy::doEmergencyStop()
{
  this->flightState->stopMotors();
}

void ControllerProxy::startVideoStream()
// makes sure all prerequisites are fulfilled. Then starts VideoController thread
// VideoController requires the InteractiveVisionState
{
  this->couplingState->connect(); // make sure command interface is connected
  this->couplingState->activateInteraction(); // make sure we are in interactive mode
  this->couplingState->activateVideo(); // start Tello sending the video stream

  if (!this->videoController->isConnected() )
    this->videoController->connect();

  this->videoController->start(); // start VideoController thread
}

void ControllerProxy::stopVideoStream()
{
  this->videoController->stop(); // stop VideoController thread
  this->couplingState->deactivateVideo(); // stop Tello sending the video stream

  if (this->videoController->isConnected() )
    this->videoController->disconnect();
}

void ControllerProxy::startSensorStream()
// makes sure all prerequisites are fulfilled. Then starts SensorController thread
// SensorController requires the InteractiveState
{
  this->couplingState->connect(); // make sure command interface is connected
  this->couplingState->activateInteraction(); // make sure we are in interactive mode

  if (!this->sensorController->isConnected() )
    this->sensorController->connect();

  this->sensorController->start(); // start SensorController thread
}

void ControllerProxy::stopSensorStream()
{
  this->sensorController->stop(); // stop SensorController thread

  if (this->sensorController->isConnected() )
    this->sensorController->disconnect();
}

MotionExecuter::MotionExecuter(CommunicationInterface* cmdInterface) :
commandInterface(cmdInterface)
{ }

void MotionExecuter::move(ClosedLoopTranslation& motion)
// executes a defined translation. If
{
  this->executing = true;
  Job translationJob = this->makeTranslationJob(motion.movement);

  int iTry = 0;
  while (iTry <= REPETITIONS_FAILED_JOBS)
  {
    iTry++;
    this->execute(translationJob);
    if (translationJob.sent && translationJob.response == POSITIVE_FEEDBACK_MSG)
    {
      motion.done = true;
      break;
    }
  }
  this->executing = false;
}

void MotionExecuter::move(ClosedLoopRotation& motion)
// add a defined motion and perform the resulting jobs
// returns true upon successful operation
// note: movement is performed first, from the local COS of the drone
// followed by the rotation
{
  this->executing = true;
  Job rotationJob = this->makeRotationJob(motion.rotation);

  int iTry = 0;
  while (iTry <= REPETITIONS_FAILED_JOBS)
  {
    iTry++;
    this->execute(rotationJob);
    if (rotationJob.sent && rotationJob.response == POSITIVE_FEEDBACK_MSG)
    {
      motion.done = true;
      break;
    }
  }
  this->executing = false;
}

void MotionExecuter::move(OpenLoopMotion& motion)
// add a continuous motion and perform the resulting job
// returns true upon successful operation
// warning: The continuous motion is executed until interrupted by another!
{
  this->executing = true;
  Job continuousJob = this->makeContinuousJob(motion);

  int iTry = 0;
  while (iTry <= REPETITIONS_FAILED_JOBS)
  {
    iTry++;
    this->execute(continuousJob);
    if (continuousJob.sent && continuousJob.response == POSITIVE_FEEDBACK_MSG)
    {
      motion.done = true;
      break;
    }
  }
  this->executing = false;
}

bool MotionExecuter::isExecuting()
{
  return this->executing;
}

void MotionExecuter::execute(Job& job)
{
  const uint8_t* command = reinterpret_cast<const uint8_t*>(job.command.c_str() );
  if (this->commandInterface->sendData(command) > 0)
    job.sent = true;
  else
  {
    job.sent = false;
    return;
  }

  if (this->commandInterface->receiveData(BLOCKING_TIME_RECEIVE) > 0) // check if got response
    job.response = (char*)this->commandInterface->getData(); // get response
  else
    job.response = ""; // must be set, as a failed job can be executed again!
}

Job MotionExecuter::makeRotationJob(RotationPoint rotation)
{
  this->unitConverter.convert(&rotation, TARGET_UNIT_ROTATION);

  Job rotationJob;
  rotationJob.command = this->createYawCommand(rotation);

  return rotationJob;
}

Job MotionExecuter::makeTranslationJob(TranslationPoint movement)
{
  this->unitConverter.convert(movement, TARGET_UNIT_TRANSLATION);

  Job movementJob;
  movementJob.command = this->createMovementCommand(movement);

  return movementJob;
}

Job MotionExecuter::makeContinuousJob(OpenLoopMotion motion)
{
  Job continuousJob;
  continuousJob.command = this->createVelocityCommand(motion);

  return continuousJob;
}

string MotionExecuter::createYawCommand(const RotationPoint& rotation) const
{
  int yaw = this->limitMagnitude(rotation.z, MIN_ROTATION_DEG, MAX_ROTATION_DEG);

  if (yaw >= 0)
    return "cw " + to_string(yaw);
  else
    return "ccw " + to_string(abs(yaw));
}

string MotionExecuter::createMovementCommand(const TranslationPoint& movement) const
{
  int x = 0, y = 0, z = 0;

  if (movement.x > 0 )
    x = this->limitMagnitude(movement.x, MIN_TRANSLATION_CM, MAX_TRANSLATION_CM);
  if (movement.y > 0 )
    y = this->limitMagnitude(movement.y, MIN_TRANSLATION_CM, MAX_TRANSLATION_CM);
  if (movement.z > 0 )
    z = this->limitMagnitude(movement.z, MIN_TRANSLATION_CM, MAX_TRANSLATION_CM);

  return "go " + to_string(x) + " " + to_string(y) + " " + to_string(z) + " " + to_string(SPEED_CM_PER_S);
}

string MotionExecuter::createVelocityCommand(const OpenLoopMotion& velocity) const
{
  int x = this->limitValue(velocity.x, MIN_RELATIVE_SPEED, MAX_RELATIVE_SPEED);
  int y = this->limitValue(velocity.y, MIN_RELATIVE_SPEED, MAX_RELATIVE_SPEED);
  int z = this->limitValue(velocity.z, MIN_RELATIVE_SPEED, MAX_RELATIVE_SPEED);
  int yaw = this->limitValue(velocity.yaw, MIN_RELATIVE_SPEED, MAX_RELATIVE_SPEED);

  return "rc " + to_string(y) + " " + to_string(x) + " " + to_string(z) + " " + to_string(yaw);
}

int MotionExecuter::limitValue(int value, int lowerBorder, int upperBorder) const
// adjusts value to the upper and lower limits
{
  return min(max (value, lowerBorder), upperBorder);
}

int MotionExecuter::limitMagnitude(int value, int absLowerBorder, int absUpperBorder) const
// adjusts the magnitude of value to the magnitude of the given limits and keeps the sign
// e.g. value = -5, absLowerBorder = 10 , absUpperBorder = 100 => returns -10
{
  int limitedMagnitude = this->limitValue(abs(value), abs(absLowerBorder), abs(absUpperBorder) );
  return copysign(limitedMagnitude, value);
}
