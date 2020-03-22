#include "VideoStream.hpp"

using namespace std;

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
  if ( !this->openCodec() ) printf("Error opening the codec! \n");

  this->parserContext = av_parser_init(this->codecType);
  if (!this->parserContext) printf("Error initializing parser! \n");

  this->frame = av_frame_alloc();
  if (!this->frame) printf("Error allocating frame! \n");
}

bool Decoder::openCodec()
// returns true/false upon successful/unsucćessful opening of the H264 codec
{
  AVCodec* codec = avcodec_find_decoder(this->codecType); // returns NULL if codec type not found
  if (!codec) return false;

  this->codecContext = avcodec_alloc_context3(codec); // allocates an AVCodecContext and set its fields to default values
  if (codec->capabilities & CODEC_CAP_TRUNCATED) this->codecContext->flags |= CODEC_FLAG_TRUNCATED;

  if (avcodec_open2(codecContext, codec, nullptr) < 0) // returns zero on success, a negative value on error
    return false;
  else
    return true;
}

bool Decoder::parseEncodedData(unsigned char* encodedData, int size)
// parses the data and returns true if parsing has finished or false if additional encoded data is required.
{
  int bufferSize = 0; // will be set to size of parsed buffer or 0 if more data is required
  int nBytes = av_parser_parse2(this->parserContext, this->codecContext, &this->parserBuffer, &bufferSize, encodedData, size, 0, 0, AV_NOPTS_VALUE);

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
  int bytesDecoded = avcodec_decode_video2(this->codecContext, this->frame, &gotPicture, &this->avPacket);

  return gotPicture != 0;
}

void Decoder::createAvPacket(unsigned char* parsedData, int size)
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
