#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <thread>
#include <memory.h>
#include <sys/time.h>
#include <queue>
#include "include/RTSP.hh"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

extern "C" {
#include <rockchip/rockchip_rga.h>
}

using namespace std;


unsigned long get_time(void)
{
	struct timeval ts;
	gettimeofday(&ts, NULL);
	return (ts.tv_sec * 1000 + ts.tv_usec / 1000);
}

RTSPFramedSource::RTSPFramedSource(UsageEnvironment& env):
  FramedSource(env),
  white_pos(0),
  fOutputBuffer(NULL),
  fMaxOfFrameToSend(1),fTotOfFrameToSend(0)
{
}

inline void RTSPFramedSource::registerOutputInterest(void) {
	fOutputBuffer=fTo;
	fTotOfFrameToSend=0;//缓冲中的图像帧数
}

inline void RTSPFramedSource::reset(void)
{
  	fOutputBuffer=NULL;
  	fTotOfFrameToSend=0;//缓冲中的图像帧数
}

inline void RTSPFramedSource::convernt_to_OutputBuffer(void)
{
	static int frame_size = 0;
	frame_size = 640 * 480 * 1.5*sizeof(unsigned char);
#if 0 //这一段是模拟推YUYV
	
	int line_size= 640 * 2*sizeof(unsigned char);
	memset(fOutputBuffer, 0,480*line_size);
	if(white_pos>479) white_pos=0;
	memset(fOutputBuffer+line_size*white_pos++, 255,line_size);
#else//下面模拟推mat
	int line_size= 640 * 3*sizeof(unsigned char);
	cv::Mat tmp;
	tmp=cv::Mat::zeros(640,480,CV_8UC3);
	if(white_pos>479) white_pos=0;
		memset(tmp.data+line_size*white_pos++, 255,line_size);
#endif

	RockchipRga *rga;
	rga = RgaCreate();
	if (!rga) {
		std::cout << "rgaCreate error!\n" << std::endl;
		return;
	}
	// std::cout << "convernt_to_OutputBuffer!\n" << std::endl;
	rga->ops->initCtx(rga); 
	rga->ops->setRotate(rga, RGA_ROTATE_NONE);
	rga->ops->setSrcFormat(rga, V4L2_PIX_FMT_YUYV, 640, 480);
	// rga->ops->setSrcFormat(rga, V4L2_PIX_FMT_YUYV, 640, 480);//推YUYV，v4l2
	rga->ops->setSrcFormat(rga, V4L2_PIX_FMT_RGB24, 640, 480);//推mat
	rga->ops->setDstFormat(rga, V4L2_PIX_FMT_NV12, 640, 480);

	
	unsigned char *frame_nv12 = NULL;
	
	frame_nv12 = (unsigned char *)malloc(frame_size);
	if (!frame_nv12)
		return;
	rga->ops->setDstBufferPtr(rga, frame_nv12);
	// rga->ops->setSrcBufferPtr(rga, (unsigned char *) camera_buffers[index].start);//推v4l2
	// rga->ops->setSrcBufferPtr(rga, (unsigned char *)fOutputBuffer);//推YUYV
	rga->ops->setSrcBufferPtr(rga, (unsigned char *)tmp.data);//推mat
	int ret = rga->ops->go(rga);
	if (ret)
	{
		printf("rga->ops->go fail! \n");
	}
  	else
  	{
    		//转换完成
			memcpy(fOutputBuffer, frame_nv12, frame_size);
    		fOutputBuffer += frame_size;
    		fTotOfFrameToSend++;
  	}
	free(frame_nv12);
	RgaDestroy(rga);
}

void RTSPFramedSource::doGetNextFrame()
{
	// std::cout << "doGetNextFrame!\n" << std::endl;
	long bigin = get_time();

  	registerOutputInterest();

  	while(fTotOfFrameToSend<fMaxOfFrameToSend)
  	{
    		convernt_to_OutputBuffer();
  	}
	long end = get_time();

 	fNumTruncatedBytes = 0;

  	reset();
	afterGetting(this);
}

