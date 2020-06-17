/**********
This library is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the
Free Software Foundation; either version 2.1 of the License, or (at your
option) any later version. (See <http://www.gnu.org/copyleft/lesser.html>.)

This library is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License
along with this library; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
**********/
// "liveMedia"
// Copyright (c) 1996-2014 Live Networks, Inc.  All rights reserved.
// A 'ServerMediaSubsession' object that creates new, unicast, "RTPSink"s
// on demand, from a H264 video file.
// Implementation

#include "include/DD_H264VideoFileServerMediaSubsession.hh"
#include "H264VideoRTPSink.hh"
#include "include/StreamEncoder.hh"
#include "include/RTSP.hh"
#include "H264VideoStreamFramer.hh"
#include <iostream>

DD_H264VideoFileServerMediaSubsession*
DD_H264VideoFileServerMediaSubsession::createNew(UsageEnvironment& env, char const* fileName, Boolean reuseFirstSource)
{
  	return new DD_H264VideoFileServerMediaSubsession(env, fileName, reuseFirstSource);
}

DD_H264VideoFileServerMediaSubsession::DD_H264VideoFileServerMediaSubsession(UsageEnvironment& env, char const* fileName, Boolean reuseFirstSource)
  : OnDemandServerMediaSubsession(env, reuseFirstSource),
    fAuxSDPLine(NULL),camera(NULL)
{

}

DD_H264VideoFileServerMediaSubsession::~DD_H264VideoFileServerMediaSubsession()
{
  	delete[] fAuxSDPLine;
}

char const* DD_H264VideoFileServerMediaSubsession::getAuxSDPLine(RTPSink* rtpSink, FramedSource* inputSource)
{
  	char const* auxSDPLine = rtpSink->auxSDPLine();
  	if (auxSDPLine != NULL)
  	{
    		return auxSDPLine;
  	}
  	else
  	{
    		char const* fmtpFmt = "a=fmtp:%d packetization-mode=1"
      					";profile-level-id=000000"
      					";sprop-parameter-sets=H264\r\n";

    		unsigned fmtpFmtSize = strlen(fmtpFmt)+3/* max char len */;

    		char* fmtp = new char[fmtpFmtSize];
    		delete[] fAuxSDPLine;
			//缺少下面这一句，调了3天，F**K!!
			memcpy(fmtp,fmtpFmt,fmtpFmtSize*sizeof(char));
    		fAuxSDPLine = fmtp;
    		return fAuxSDPLine;
  	}
}

FramedSource* DD_H264VideoFileServerMediaSubsession::createNewStreamSource(unsigned /*clientSessionId*/, unsigned& estBitrate)
{
	std::cout << "createNewStreamSource!\n" << std::endl;
  	RTSPFramedSource *rtsp_;

  	estBitrate = 10000; // kbps, estimate
 
	rtsp_ = new RTSPFramedSource(envir());
  	if(rtsp_==NULL)
		return NULL;

  	if(camera!=NULL)
  	{
    		camera=NULL;
  	}
  	camera=rtsp_;

  	StreamEncoder* fileSource = StreamEncoder::createNew(envir(),rtsp_,1000000);
  	if (fileSource == NULL)
		return NULL;
	std::cout << "111!\n" << std::endl;
	FramedSource*tmp=H264VideoStreamFramer::createNew(envir(), fileSource);
	std::cout << "111!\n" << std::endl;
 	return tmp;
}

RTPSink* DD_H264VideoFileServerMediaSubsession::createNewRTPSink(Groupsock* rtpGroupsock,
		   unsigned char rtpPayloadTypeIfDynamic,
		   FramedSource* /*inputSource*/)
{
  	return H264VideoRTPSink::createNew(envir(), rtpGroupsock, rtpPayloadTypeIfDynamic);
}

void DD_H264VideoFileServerMediaSubsession::startStream(unsigned clientSessionId,
						void* streamToken,
						TaskFunc* rtcpRRHandler,
						void* rtcpRRHandlerClientData,
						unsigned short& rtpSeqNum,
						unsigned& rtpTimestamp,
						ServerRequestAlternativeByteHandler* serverRequestAlternativeByteHandler,
						void* serverRequestAlternativeByteHandlerClientData)
{
	std::cout << "Subsession::startStream!\n" << std::endl;
  	StreamState* streamState = (StreamState*)streamToken;

	OnDemandServerMediaSubsession::startStream(clientSessionId,streamToken,rtcpRRHandler,
		rtcpRRHandlerClientData,rtpSeqNum,rtpTimestamp,serverRequestAlternativeByteHandler,
		serverRequestAlternativeByteHandlerClientData);
}

void DD_H264VideoFileServerMediaSubsession::pauseStream(unsigned /*clientSessionId*/, void* streamToken)
{
  	StreamState* streamState = (StreamState*)streamToken;
					
  	OnDemandServerMediaSubsession::pauseStream(0,streamToken);
}
