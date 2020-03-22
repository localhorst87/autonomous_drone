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
}

#include <stdio.h>
#include <iostream>

class Decoder
{
  private:
    static const AVCodecID codecType = AV_CODEC_ID_H264;
    AVCodecContext* codecContext = nullptr; // info struct about the chosen codec
    AVCodecParserContext* parserContext = nullptr; // info struct regarding the parser (contains the parser itself!)
    AVFrame* frame; // target frame for decoded data
    AVPacket avPacket;
    unsigned char* parserBuffer; // will be	set to pointer to parsed buffer or NULL if more data is required
    bool isParsingDone = false;

  private:
    void configureDecoder();
    void createAvPacket(unsigned char*, int);
    bool openCodec();

  public:
    Decoder();
    ~Decoder();
    bool decodeParsedData();
    bool parseEncodedData(unsigned char*, int);
    AVFrame getFrame();
};

#endif
