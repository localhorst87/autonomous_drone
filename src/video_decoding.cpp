#include "VideoStream.hpp"

using namespace std;

RgbImage::RgbImage(AVFrame* rgbFrame) :
  dataLinesize(rgbFrame->linesize[0]),
  width(rgbFrame->width),
  height(rgbFrame->height),
  nPoints(rgbFrame->linesize[0]*rgbFrame->height)
{
  this->checkFrameFormat(rgbFrame); // throws exception if format is not valid
  this->rawData = (uint8_t*) malloc(this->nPoints);
  memcpy(this->rawData, rgbFrame->data[0], this->nPoints);
}

RgbImage& RgbImage::operator=(const RgbImage& rgbImage)
{
  this->dataLinesize = rgbImage.dataLinesize;
  this->width = rgbImage.width;
  this->height = rgbImage.height;
  this->nPoints = rgbImage.nPoints;
  this->rawData = (uint8_t*) malloc(this->nPoints);
  memcpy(this->rawData, rgbImage.rawData, this->nPoints);
}

RgbImage::RgbImage()
{ }

RgbImage::RgbImage(const RgbImage& rgbImage) :
  dataLinesize(rgbImage.dataLinesize),
  width(rgbImage.width),
  height(rgbImage.height),
  nPoints(rgbImage.nPoints)
{
  this->rawData = (uint8_t*) malloc(this->nPoints);
  memcpy(this->rawData, rgbImage.rawData, this->nPoints);
}

RgbImage::~RgbImage()
{
  if (this->rawData)
    free(this->rawData);
}

void RgbImage::checkFrameFormat(AVFrame* frame)
// checks if the format of the AVFrame is RGB24. If not, throws an error!
{
  if (frame->format != AV_PIX_FMT_RGB24)
    throw invalid_argument("Frame is not of type AV_PIX_FMT_RGB24");
}

ThreeDimByteVector RgbImage::getRgbPixels()
// returns a three-dimensional vector of size width x height x 3. So each pixel in the picture
// contains an uint8_t - vector with {red, green, blue} data inside.
{
  ThreeDimByteVector rgbPixels(this->width, vector<ByteVector>( this->height, ByteVector(3) ) );

  for (int i = 0; i < this->nPoints; i++)
  {
    auto pixelIdx = calcPixelIndices(i, WIDTH_HEIGHT_COLOR);
    rgbPixels[ get<0>(pixelIdx) ][ get<1>(pixelIdx) ][ get<2>(pixelIdx) ] = this->rawData[i];
  }

  return rgbPixels;
}

ThreeDimByteVector RgbImage::getRgbPlanes()
// returns a three-dimensional vector of size 3 x width x height. So each of the colors red,
// green and blue contains a plane of a uint8_t - vector of the picture's size (width x height)
{
  ThreeDimByteVector rgbPlanes(3, vector<ByteVector>( this->width, ByteVector(this->height) ) );

  for (int i = 0; i < this->nPoints; i++)
  {
    auto pixelIdx = calcPixelIndices(i, COLOR_WIDTH_HEIGHT);
    rgbPlanes[ get<0>(pixelIdx) ][ get<1>(pixelIdx) ][ get<2>(pixelIdx) ] = this->rawData[i];
  }

  return rgbPlanes;
}

tuple<int, int, int> RgbImage::calcPixelIndices(int dataIdx, RgbPixelIndexOrder idxOrder)
// returns a tuple of the width-, heigth- and color-index of the pixel,
// as a function of the position inside the raw data array
// 0: width-index, 1: height-index, 2: color-index
{
  int heightIdx {dataIdx / this->dataLinesize};
  int widthIdx { (dataIdx - heightIdx * this->dataLinesize) / 3 };
  int colorIdx {dataIdx % 3};

  if (idxOrder == WIDTH_HEIGHT_COLOR)
    return make_tuple(widthIdx, heightIdx, colorIdx);
  else if (idxOrder == COLOR_WIDTH_HEIGHT)
    return make_tuple(colorIdx, widthIdx, heightIdx);
  else
    throw invalid_argument("Uknown RgbPixelIndexOrder argument");
}

Decoder::Decoder()
{
  avcodec_register_all(); // Registers all the codecs, parsers and bitstream filters enabled at configuration time
  this->configureDecoder();
}

Decoder::~Decoder()
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

void Decoder::configureDecoder()
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

bool Decoder::openCodec()
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

int Decoder::parseEncodedData(uint8_t* encodedData, int size)
// parses the data and returns true if parsing has finished or false if additional encoded data is required.
// Important: For useful output a key frame must be sent by the drone's camera. This can take up to
// a few seconds after starting to receive video stream data!
{
  int nParsedBytes = av_parser_parse2(this->parserContext, this->codecContext,
                                  &this->avPacket.data, &this->avPacket.size,
                                  encodedData, size, 0, 0, AV_NOPTS_VALUE);

  return nParsedBytes;
}

bool Decoder::decodeParsedData()
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

bool Decoder::readyForDecoding() const
// indicates if data can be decoded after parsing
{
  return this->avPacket.size > 0;
}

AVFrame Decoder::getFrame() const
// returns the decoced frame
{
  return *this->frame;
}

Converter::Converter()
{
  this->destinationFrame = av_frame_alloc();
  this->setSwsContext();
  this->allocateBuffer();
}

Converter::~Converter()
{
  if (this->buffer)
    free(this->buffer);
  av_frame_free(&this->destinationFrame);
}

void Converter::setSwsContext()
// sets the color conversion and scaling context from the given parameters
{
    this->swsContext = sws_getContext(SOURCE_WIDTH, SOURCE_HEIGHT, SOURCE_FORMAT, this->destinationWidth, this->destinationHeight,
                                      DESTINATION_FORMAT, SWS_BILINEAR, nullptr, nullptr, nullptr);
}

int Converter::allocateBuffer()
// get required buffer size and allocate buffer with this size
{
  int size = av_image_fill_arrays(this->destinationFrame->data, this->destinationFrame->linesize,
                                  nullptr, DESTINATION_FORMAT, this->destinationWidth, this->destinationHeight, 1);
  this->buffer = (uint8_t*) malloc(size);

  return size;
}

void Converter::setFrameMetaData()
// sets width, height and format of the destination frame
{
  this->destinationFrame->width = this->destinationWidth;
  this->destinationFrame->height = this->destinationHeight;
  this->destinationFrame->format = DESTINATION_FORMAT;
}

void Converter::setScaleFactor(float scaleFactor)
// sets width and height of destination frame according to the given scale factor
// with respect to the original frame.
{
  if (scaleFactor > SCALE_MAX || scaleFactor < SCALE_MIN)
  {
    scaleFactor = min(SCALE_MAX, max(SCALE_MIN, scaleFactor) );
    printf("scale factor of %f is out of range! Adjusted scaleFactor to range of [%f, %f]. \n",
            scaleFactor, SCALE_MIN, SCALE_MAX);
  }

  this->destinationWidth = SOURCE_WIDTH * scaleFactor;
  this->destinationHeight = SOURCE_HEIGHT * scaleFactor;
}

RgbImage Converter::convert(const AVFrame& frame)
// converts the YUV420 frame to and RgbImage and returns the latter
{
  av_image_fill_arrays(this->destinationFrame->data, this->destinationFrame->linesize,
                        this->buffer, DESTINATION_FORMAT, this->destinationWidth, this->destinationHeight, 1);

  sws_scale(this->swsContext, frame.data, frame.linesize, 0, this->destinationHeight,
            this->destinationFrame->data, this->destinationFrame->linesize);
  this->setFrameMetaData();

  RgbImage rgbImage {this->destinationFrame};

  return rgbImage;
}
