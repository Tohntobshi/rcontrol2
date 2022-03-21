#include "videoEncoder.h"
#include <iostream>

VideoEncoder::VideoEncoder(int w, int h, int bitRate, int fps):
width(w),
height(h)
{
    swsEncCont = sws_getContext(width, height, AV_PIX_FMT_BGR24, width, height, AV_PIX_FMT_YUV420P, SWS_BICUBIC, nullptr, nullptr, nullptr);
    AVCodec *codec = avcodec_find_encoder(codecId);
    if (codec == nullptr)
    {
        std::cout << "codec not found\n";
    }
    enC = avcodec_alloc_context3(codec);
    enC->width = width;
    enC->height = height;
    enC->pix_fmt = AV_PIX_FMT_YUV420P;
    enC->bit_rate = bitRate;
    enC->time_base = (AVRational){1, fps};
    enC->gop_size = 0;
    if (avcodec_open2(enC, codec, nullptr) < 0)
    {
        std::cout << "cannot open codec\n";
    }
}

VideoEncoder::~VideoEncoder()
{
    sws_freeContext(swsEncCont);
    avcodec_free_context(&enC);
}

std::tuple<uint8_t *, int> VideoEncoder::encode(uint8_t * data, int size)
{
    uint8_t * rawData = (uint8_t*)av_malloc(size);
    memcpy(rawData, data, size);
    AVFrame *initialFrame = av_frame_alloc();
    initialFrame->width = width;
    initialFrame->height = height;
    initialFrame->format = AV_PIX_FMT_BGR24;
    av_image_fill_arrays(initialFrame->data, initialFrame->linesize, rawData, AV_PIX_FMT_BGR24, width, height, 1);

    AVFrame *destFrame = av_frame_alloc();
    destFrame->width = width;
    destFrame->height = height;
    destFrame->format = AV_PIX_FMT_YUV420P;
    destFrame->pts = pts;
    pts++;
    av_image_alloc(destFrame->data, destFrame->linesize, width, height, AV_PIX_FMT_YUV420P, 1);

    sws_scale(swsEncCont, initialFrame->data, initialFrame->linesize, 0, height, destFrame->data, destFrame->linesize);

    int sendFrRes = avcodec_send_frame(enC, destFrame);

    av_freep(&initialFrame->data[0]);
    av_freep(&destFrame->data[0]);
    av_frame_free(&initialFrame);
    av_frame_free(&destFrame);

    if (sendFrRes != 0)
    {
        if (sendFrRes == AVERROR(EAGAIN))
        {
            std::cout << "send frame eagain\n";
        }
        else
        {
            std::cout << "send frame error\n";
        }
    }

    AVPacket *pak = av_packet_alloc();
    int recPkRes = avcodec_receive_packet(enC, pak);
    if (recPkRes != 0)
    {
        if (recPkRes == AVERROR(EAGAIN))
        {
            std::cout << "receive packet eagain\n";
        }
        else
        {
            std::cout << "receive packet error\n";
        }
        av_packet_free(&pak);
        return { nullptr, 0 };
    }
    int outputSize = pak->size;
    uint8_t * outputData = new uint8_t[outputSize];
    memcpy(outputData, pak->data, outputSize);
    av_packet_free(&pak);
    return { outputData, outputSize };
}