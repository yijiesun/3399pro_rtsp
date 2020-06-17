#ifndef RGB2YUV_H
#define RGB2YUV_H

typedef unsigned char uint8_t;

#define COLORSIZE 256

class Rgb2YUV
{
public:

    typedef struct RGB
    {
           unsigned char r;
           unsigned char g;
           unsigned char b;
    }RGB;

    Rgb2YUV();

    /**
     * @brief RGB2YUV420
     * @param rgbBufIn   rgb数据输入
     * @param yuvBufOut  yuv数据输出
     * @param nWidth
     * @param nHeight
     */
    void RGB2YUV420(uint8_t* rgbBufIn, uint8_t* yuvBufOut, int nWidth, int nHeight);

private:
    unsigned short Y_R[COLORSIZE],Y_G[COLORSIZE],Y_B[COLORSIZE],U_R[COLORSIZE],U_G[COLORSIZE],U_B[COLORSIZE],V_G[COLORSIZE],V_B[COLORSIZE]; //查表数组V_R[COLORSIZE]
    void table_init();

};

#endif // RGB2YUV_H
