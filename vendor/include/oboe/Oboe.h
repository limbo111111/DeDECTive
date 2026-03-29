#ifndef OBOE_H
#define OBOE_H

#include <stdint.h>
#include <string>

namespace oboe {

enum class DataCallbackResult { Continue, Stop };
enum class Direction { Output, Input };
enum class PerformanceMode { None, LowLatency, PowerSaving };
enum class SharingMode { Exclusive, Shared };
enum class AudioFormat { Invalid, Unspecified, I16, Float };
enum class ChannelCount { Unspecified, Mono, Stereo };
enum class Result { OK, ErrorInternal };

class AudioStream;

class AudioStreamDataCallback {
public:
    virtual ~AudioStreamDataCallback() = default;
    virtual DataCallbackResult onAudioReady(AudioStream* oboeStream, void* audioData, int32_t numFrames) = 0;
};

class AudioStream {
public:
    virtual ~AudioStream() = default;
    virtual Result requestStart() = 0;
    virtual Result requestStop() = 0;
    virtual Result close() = 0;
    virtual AudioStreamDataCallback* getDataCallback() const = 0;
};

class AudioStreamBuilder {
public:
    AudioStreamBuilder* setDirection(Direction direction);
    AudioStreamBuilder* setPerformanceMode(PerformanceMode performanceMode);
    AudioStreamBuilder* setSharingMode(SharingMode sharingMode);
    AudioStreamBuilder* setFormat(AudioFormat format);
    AudioStreamBuilder* setChannelCount(ChannelCount channelCount);
    AudioStreamBuilder* setSampleRate(int32_t sampleRate);
    AudioStreamBuilder* setDataCallback(AudioStreamDataCallback* dataCallback);
    Result openStream(AudioStream** stream);
};

const char* convertToText(Result returnCode);

} // namespace oboe
#endif
