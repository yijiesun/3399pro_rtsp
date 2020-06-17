PROJECT(ZSXL)
CMAKE_MINIMUM_REQUIRED(VERSION 3.5.1)
set(CMAKE_CXX_STANDARD 14)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")

set(RTSP_SRCS 

${CMAKE_CURRENT_SOURCE_DIR}/src/rtsp/DD_H264VideoFileServerMediaSubsession.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/rtsp/StreamEncoder.cpp 
${CMAKE_CURRENT_SOURCE_DIR}/src/rtsp/testOnDemandRTSPServer.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/rtsp/RTSP.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/rtsp/demo.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/rgb2yuv/rgb2yuv.cpp
)
# opencv
find_package(OpenCV REQUIRED)
message(STATUS ${OpenCV_LIBRARY_DIRS})
#include
include_directories(
      ${OpenCV_INCLUDE_DIRS} 
      /usr/include
      /usr/include/ffmpeg
      /usr/include/groupsock
      /usr/include/liveMedia
      /usr/include/BasicUsageEnvironment
      /usr/include/UsageEnvironment
)


#exe
# add_executable(zsxl ${SRCS})
# target_link_libraries(zsxl  ${OpenCV_LIBS} pthread )

LINK_DIRECTORIES(/usr/lib64)
add_executable(rtsp ${RTSP_SRCS})
target_link_libraries(rtsp ${OpenCV_LIBS}  pthread -lrockchip_mpp  -lrockchip_rga swscale avformat avcodec UsageEnvironment groupsock BasicUsageEnvironment liveMedia)
