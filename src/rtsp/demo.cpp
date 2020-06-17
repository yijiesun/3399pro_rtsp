#include "./include/DD_H264VideoFileServerMediaSubsession.hh"
#include <stdio.h>  
#include <thread>  
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
rtsp_process rtsp_;
void *rtsp_thread(void *threadarg)
{
    rtsp_.process();
}
int main(int argc, char *argv[])
{
    pthread_t threads_rtsp;
    //多线程
    pthread_create(&threads_rtsp, NULL, rtsp_thread, NULL);

    while(1)
    {
        //do something else
        ;
    }
    
    return 0;
}