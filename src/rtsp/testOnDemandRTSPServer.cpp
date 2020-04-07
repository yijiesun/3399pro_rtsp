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
// Copyright (c) 1996-2014, Live Networks, Inc.  All rights reserved
// A test program that demonstrates how to stream - via unicast RTP
// - various kinds of file on demand, using a built-in RTSP server.
// main program
// #include "../process/include.h"
// #include "../menu_config/menu_config.h"
// #include "../funs/funs.h"
#include "./include/DD_H264VideoFileServerMediaSubsession.hh"
#include <stdio.h>  
#include <iostream>
using namespace std;


unsigned long rtsp_process::get_time(void)
{
	struct timeval ts;
	gettimeofday(&ts, NULL);
	return (ts.tv_sec * 1000 + ts.tv_usec / 1000);
}

void rtsp_process::init()
{
        // Begin by setting up our usage environment:
    TaskScheduler* scheduler = BasicTaskScheduler::createNew();
    env = BasicUsageEnvironment::createNew(*scheduler);
    Boolean reuseFirstSource = True;
    UserAuthenticationDatabase* authDB = NULL;
  #ifdef ACCESS_CONTROL
    // To implement client access control to the RTSP server, do the following:
    authDB = new UserAuthenticationDatabase;
    authDB->addUserRecord("username1", "password1"); // replace these with real strings
    // Repeat the above with each <username>, <password> that you wish to allow
    // access to the server.
  #endif

    // Create the RTSP server:
    RTSPServer* rtspServer = RTSPServer::createNew(*env, 8554, authDB);
    if (rtspServer == NULL) {
      *env << "Failed to create RTSP server: " << env->getResultMsg() << "\n";
      exit(1);
    }

    char const* descriptionString
      = "Session streamed by \"testOnDemandRTSPServer\"";

    // Set up each of the possible streams that can be served by the
    // RTSP server.  Each such stream is implemented using a
    // "ServerMediaSession" object, plus one or more
    // "ServerMediaSubsession" objects for each audio/video substream.
    // A H.264 video elementary stream:
    {
      char const* streamName = "h264ESVideoTest";
      char const* inputFileName = "test.264";
      ServerMediaSession* sms
        = ServerMediaSession::createNew(*env, streamName, streamName,
                descriptionString);
      sms->addSubsession(DD_H264VideoFileServerMediaSubsession
            ::createNew(*env, inputFileName, reuseFirstSource));
      rtspServer->addServerMediaSession(sms);

      announceStream(rtspServer, sms, streamName, inputFileName);
    }
}
int rtsp_process::process() {
  init();
  eventLoopWatchVariable=0;
  env->taskScheduler().doEventLoop(&eventLoopWatchVariable); // does not return

  return 0; // only to prevent compiler warning
}

//有时候sms识别ip不上，收流地址是rtsp://ip:8554/h264ESVideoTest
void rtsp_process::announceStream(RTSPServer* rtspServer, ServerMediaSession* sms,
			   char const* streamName, char const* inputFileName) {
  char* url = rtspServer->rtspURL(sms);
  UsageEnvironment& env = rtspServer->envir();
  env << "\n\"" << streamName << "\" stream, from the file \""
      << inputFileName << "\"\n";
  env << "Play this stream using the URL \"" << url << "\"\n";
  delete[] url;
}
