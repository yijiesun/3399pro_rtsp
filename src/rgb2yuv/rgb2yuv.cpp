#include "rgb2yuv.h"

Rgb2YUV::Rgb2YUV()
{
    table_init();
}

//表的初始化
void Rgb2YUV::table_init()
{
    int i;
    for(i = 0; i < COLORSIZE; i++)
    {
        Y_R[i] = (i * 1224) >> 12; //Y对应的查表数组0.2988
        Y_G[i] = (i * 2404) >> 12;  //0.5869
        Y_B[i] = (i * 469)  >> 12; //0.1162
        U_R[i] = (i * 692)  >> 12; //U对应的查表数组0.1688
        U_G[i] = (i * 1356) >> 12;  //0.3312
        U_B[i] = i /*(* 2048) */>> 1; //0.5
     //   V_R[i] = (i * 2048) >> 12; ///V对应的查表数组
        V_G[i] = (i * 1731) >> 12;  //0.4184
        V_B[i] = (i * 334)  >> 12; //0.0816
    }
}

void Rgb2YUV::RGB2YUV420(uint8_t *rgbBufIn, uint8_t *yuvBufOut , int nWidth, int nHeight)
{
    int pix = 0;
    int pixP4;

    RGB *in = (RGB *)rgbBufIn; //需要计算的原始数据
//    unsigned char out[XSIZE*YSIZE * 3/2]; //计算后的结果

    int IMGSIZE = nWidth * nHeight;

    for(int y = 0; y < nHeight; y++) //line
    {
        for(int x=0;x < nWidth;x++)//pixf
        {

//           RGB rgbByte = in[pix]; (这样取的数据经过转换后是一个垂直倒立的图像)
            RGB rgbByte = in[(nHeight-y-1)*nWidth+x]; //取得垂直方向上镜像的位置即可解决倒立问题

           //首先执行颜色互换 -- 没有这个的话  得到的YUV图像颜色不对
           uint8_t temp = rgbByte.r; //顺序调整
           rgbByte.r = rgbByte.b;
           rgbByte.b = temp;

            int     i = Y_R[rgbByte.r] + Y_G[rgbByte.g] + Y_B[rgbByte.b];
            yuvBufOut[pix]= i; //Y
            if((x%2==1)&&(y%2==1))
            {
                pixP4=(nWidth>>1) *(y>>1) + (x>>1);
                i=U_B[rgbByte.b] - U_R[rgbByte.r] - U_G[rgbByte.g]+128;

                yuvBufOut[pixP4+IMGSIZE]     = i;
                /*+  U_B[in[pix+1].b] - U_R[in[pix+1].r] - U_G[in[pix+1].g]
                +U_B[in[pix+XSIZE].b] - U_R[in[pix+XSIZE].r] - U_G[in[pix+XSIZE].g]
                +U_B[in[pix+1+XSIZE].b] - U_R[in[pix+1+XSIZE].r] - U_G[in[pix+1+XSIZE].g] )/4*/
                //U
                i=U_B[rgbByte.r] - V_G[rgbByte.g] - V_B[rgbByte.b]+128;

                yuvBufOut[pixP4 + 5 * IMGSIZE /4] = i;
                /*+U_B[in[pix+1].r] - V_G[in[pix+1].g] - V_B[in[pix+1].b]
                +U_B[in[pix+XSIZE].r] - V_G[in[pix+XSIZE].g] - V_B[in[pix+XSIZE].b]
                +U_B[in[pix+1+XSIZE].r] - V_G[in[pix+1+XSIZE].g] - V_B[in[pix+1+XSIZE].b])/4*/
                //V
            }

            pix++;
        }

    }
}
