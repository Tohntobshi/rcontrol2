#include <tuple>
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

class VideoEncoder
{
public:
    VideoEncoder(int width, int  height, int bitRate, int fps);
    std::tuple<uint8_t *, int> encode(uint8_t * data, int size);
    ~VideoEncoder();
private:
    AVCodecID codecId = AV_CODEC_ID_MPEG4;
    AVCodecContext *enC = nullptr;
    SwsContext * swsEncCont = nullptr;
    int pts = 0;
    int width = 0;
    int height = 0;
};