
#ifndef _STREAM_ENCODER_HH
#define _STREAM_ENCODER_HH

#ifndef _FRAMED_FILTER_HH
#include "FramedFilter.hh"
#endif


extern "C"
{
	#include <libavcodec/avcodec.h>
	#include <libswscale/swscale.h>
  	#include "libavutil/opt.h"
  	#include <libavutil/imgutils.h>
}

class StreamEncoder: public FramedFilter {
	public:
  		virtual void flushInput();
  		static StreamEncoder *createNew(UsageEnvironment& env,FramedSource* inputSource,
                                  unsigned inputBufferMax);
  		StreamEncoder(UsageEnvironment& env, FramedSource* inputSource,unsigned inputBufferMax);
  		virtual ~StreamEncoder();
  		virtual unsigned maxFrameSize() const;

	private:
  		virtual void doGetNextFrame();
  		virtual void doStopGettingFrames();


	private:
  		void reset();
  		void registerInputInterest(void);
  		static void continueReadProcessing(void* clientData,
            				unsigned frameSize,unsigned numTruncatedBytes,
					struct timeval presentationTime,
					unsigned durationInMicroseconds);
  		void continueReadProcessing1(unsigned frameSize,unsigned numTruncatedBytes,
					struct timeval presentationTime,
					unsigned durationInMicroseconds);
  		void encoder_to_h264(void);
  		void copy_to_outputbuffer(void);
  		void readAndProcessing(void);

	private:
  		unsigned fOutputBufferSize;
  		unsigned char* fOutputBuffer;
  		unsigned fNumValidDataBytes;
  		unsigned fMaxOfFrameToSend;
  		unsigned fTotOfFrameToSend;

  		int width;
  		int height;
  		unsigned int pictureSize;
  		AVPixelFormat srcfmt;
  		unsigned char *srcbuf;

		unsigned char *dstbuf;
		int dstsize;
};
#endif
