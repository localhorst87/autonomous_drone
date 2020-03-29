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

three_dim_byte_vector RgbImage::getRgbPixels()
// returns a three-dimensional vector of size width x height x 3. So for each pixel in the picture
// contains an uint8_t - vector with {red, green, blue} data inside.
{
  three_dim_byte_vector rgbPixels(this->width, vector<byte_vector>( this->height, byte_vector(3) ) );
  int heightIdx, widthIdx, rgbIdx;

  for (int i = 0; i < this->nPoints; ++i)
  {
    auto pixelIdx = calcPixelIndices(i, WIDTH_HEIGHT_COLOR);
    rgbPixels[ get<0>(pixelIdx) ][ get<1>(pixelIdx) ][ get<2>(pixelIdx) ] = this->rawData[i];
  }

  return rgbPixels;
}

three_dim_byte_vector RgbImage::getRgbPlanes()
// returns a three-dimensional vector of size 3 x width x height. So each of the colors red,
// green and blue contains a plane of a uint8_t - vector of the picture's size (width x height)
{
  three_dim_byte_vector rgbPlanes(3, vector<byte_vector>( this->width, byte_vector(this->height) ) );
  int heightIdx, widthIdx, rgbIdx;

  for (int i = 0; i < this->nPoints; ++i)
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
  int heightIdx = dataIdx / this->dataLinesize;
  int widthIdx = (dataIdx - heightIdx * this->dataLinesize) / 3;
  int colorIdx = dataIdx % 3;

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

  if (avcodec_open2(codecContext, codec, nullptr) < 0) // returns zero on success, a negative value on error
    return false;
  else
    return true;
}

bool Decoder::parseEncodedData(uint8_t* encodedData, int size)
// parses the data and returns true if parsing has finished or false if additional encoded data is required.
{
  int bufferSize = 0; // will be set to size of parsed buffer or 0 if more data is required
  av_parser_parse2(this->parserContext, this->codecContext, &this->parserBuffer, &bufferSize, encodedData, size, 0, 0, AV_NOPTS_VALUE);

  if (bufferSize > 0)
  {
    this->createAvPacket(this->parserBuffer, bufferSize);
    this->isParsingDone = true;
    return true;
  }
  else
  {
    this->isParsingDone = false;
    return false;
  }
}

bool Decoder::decodeParsedData()
// main function to decode parsed data
{
  if (!this->isParsingDone)
  {
    printf("No parsed data available. Parse data first! \n");
    return false;
  }

  int gotPicture = 0;
  avcodec_decode_video2(this->codecContext, this->frame, &gotPicture, &this->avPacket);

  return gotPicture != 0;
}

void Decoder::createAvPacket(uint8_t* parsedData, int size)
// creates an AVPacket, that is used to store compressed data, from parsed data
{
  av_init_packet(&this->avPacket);
  this->avPacket.data = parsedData;
  this->avPacket.size = size;
}

AVFrame Decoder::getFrame()
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
                                      DESTINATION_FORMAT, SWS_BICUBIC, nullptr, nullptr, nullptr);
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
