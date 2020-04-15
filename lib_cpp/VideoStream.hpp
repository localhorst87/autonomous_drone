/*
classes for decoding and converting the raw video bytestream
The raw data is H264 encoded. I use libav for decoding. For any information on
functionality, see the libav doc: https://libav.org/documentation/doxygen/master

Hands-on tutorials are provided in the Libav documentation as well. Another useful documentation
is provided by roxlu.com: https://www.roxlu.com/2014/039/decoding-h264-and-yuv420p-playback

Make sure to install libav for functionality before! https://libav.org/download
*/

#ifndef VIDEO_STREAM_HPP
#define VIDEO_STREAM_HPP

extern "C"
{
  #include <libavcodec/avcodec.h>
  #include <libavutil/avutil.h>
  #include <libswscale/swscale.h>
  #include <libavutil/imgutils.h>
}

#include <stdio.h>
#include <iostream>
#include <vector>
#include <tuple>
#include <cstring>
#include <stdexcept>
#include <algorithm>

typedef std::vector<std::vector<std::vector<uint8_t>>> three_dim_byte_vector;
typedef std::vector<uint8_t> byte_vector;

enum RgbPixelIndexOrder
{
  WIDTH_HEIGHT_COLOR,
  COLOR_WIDTH_HEIGHT
};

class RgbImage
{
  private:
    uint8_t* rawData = nullptr;
    int dataLinesize;
    int width;
    int height;
    int nPoints;

  private:
    void checkFrameFormat(AVFrame*);
    std::tuple<int, int, int> calcPixelIndices(int, RgbPixelIndexOrder);

  public:
    RgbImage(AVFrame*);
    RgbImage(const RgbImage&);
    RgbImage& operator=(const RgbImage&);
    RgbImage();
    ~RgbImage();
    three_dim_byte_vector getRgbPixels();
    three_dim_byte_vector getRgbPlanes();
};

class Decoder
{
  private:
    const AVCodecID CODEC_TYPE = AV_CODEC_ID_H264;
    AVCodecContext* codecContext = nullptr; // info struct about the chosen codec
    AVCodecParserContext* parserContext = nullptr; // info struct regarding the parser (contains the parser itself!)
    AVFrame* frame; // target frame for decoded data
    AVPacket avPacket;

  private:
    void configureDecoder();
    bool openCodec();

  public:
    Decoder();
    ~Decoder();
    bool decodeParsedData();
    int parseEncodedData(uint8_t*, int);
    bool readyForDecoding() const;
    AVFrame getFrame() const;
};

class Converter
{
  private:
    const AVPixelFormat DESTINATION_FORMAT = AV_PIX_FMT_RGB24; // three channel 8-bit
    const AVPixelFormat SOURCE_FORMAT = AV_PIX_FMT_YUV420P;
    const int SOURCE_WIDTH = 960;
    const int SOURCE_HEIGHT = 720;
    const float SCALE_MAX = 10;
    const float SCALE_MIN = 0.1;
    int destinationWidth = 960;
    int destinationHeight = 720;
    SwsContext* swsContext = nullptr;
    AVFrame* destinationFrame;
    uint8_t* buffer = nullptr;

  private:
    void setSwsContext();
    int allocateBuffer();
    void setFrameMetaData();

  public:
    Converter();
    ~Converter();
    void setScaleFactor(float);
    RgbImage convert(const AVFrame&);
};

#endif
