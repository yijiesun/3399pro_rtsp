
#ifndef RTSP_H_
#define RTSP_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <linux/videodev2.h>

#ifndef _FRAMED_SOURCE_HH
#include "FramedSource.hh"
#endif

extern "C"
{
	#include <libavcodec/avcodec.h>
	#include <libswscale/swscale.h>
  	#include "libavutil/opt.h"
  	#include <libavutil/imgutils.h>
}

#define CLEAR(x) memset(&(x),0,sizeof(x))


class RTSPFramedSource: public FramedSource
{
public:
	int white_pos;
	RTSPFramedSource(UsageEnvironment& env);

protected: 
  	virtual void doGetNextFrame();

private:
  	inline void reset(void);
  	inline void registerOutputInterest(void);
  	inline void convernt_to_OutputBuffer(void);

  	unsigned char* fOutputBuffer;//缓存起始地址
  	unsigned fMaxOfFrameToSend;//最大输出图像帧数
  	unsigned fTotOfFrameToSend;//图像输出总帧数

};

#endif /* V4L2_H_ */
