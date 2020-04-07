#include "include/StreamEncoder.hh"
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/ioctl.h>   
#include <sys/types.h>   
#include <sys/stat.h>   
#include <fcntl.h>   
#include <errno.h>  
#include <rockchip/rockchip_mpp.h>
#include "rockchip/rk_mpi.h"
#include "rockchip/mpp_buffer.h"
#include "rockchip/mpp_err.h"
#include "rockchip/mpp_frame.h"
#include "rockchip/mpp_meta.h"
#include "rockchip/mpp_packet.h"
#include "rockchip/mpp_task.h"



#define MPP_ALIGN(x, a)         (((x)+(a)-1)&~((a)-1))
#define mpp_malloc(type, count)  \
    (type*)malloc(sizeof(type) * (count))
#define mpp_calloc(type, count)  \
    (type*)malloc(sizeof(type) * (count))
#define mpp_free(ptr) \
    free(ptr)

#define MPP_FREE(ptr)   do { if(ptr) mpp_free(ptr); ptr = NULL; } while (0)

#define MAX_FILE_NAME_LENGTH 256

export unsigned long get_time(void);

typedef struct {
    char            file_input[MAX_FILE_NAME_LENGTH];
    char            file_output[MAX_FILE_NAME_LENGTH];
    MppCodingType   type;
    RK_U32          width;
    RK_U32          height;
    MppFrameFormat  format;
    RK_U32          debug;
    RK_U32          num_frames;

    RK_U32          have_input;
    RK_U32          have_output;
} MpiEncTestCmd;

typedef struct {
    // global flow control flag
    RK_U32 frm_eos;
    RK_U32 pkt_eos;
    RK_U32 frame_count;
    RK_U64 stream_size;

    // src and dst
    FILE *fp_input;
    FILE *fp_output;

    // base flow context
    MppCtx ctx;
    MppApi *mpi;
    MppEncPrepCfg prep_cfg;
    MppEncRcCfg rc_cfg;
    MppEncCodecCfg codec_cfg;

    // input / output
    MppBuffer frm_buf;
    MppEncSeiMode sei_mode;

    // paramter for resource malloc
    RK_U32 width;
    RK_U32 height;
    RK_U32 hor_stride;
    RK_U32 ver_stride;
    MppFrameFormat fmt;
    MppCodingType type;
    RK_U32 num_frames;

    // resources
    size_t frame_size;
    /* NOTE: packet buffer may overflow */
    size_t packet_size;

    // rate control runtime parameter
    RK_S32 gop;
    RK_S32 fps;
    RK_S32 bps;
} MpiEncTestData;

MPP_RET test_ctx_init(MpiEncTestData **data, MpiEncTestCmd *cmd)
{
    MpiEncTestData *p = NULL;
    MPP_RET ret = MPP_OK;

    if (!data || !cmd) {
        printf("invalid input data %p cmd %p\n", data, cmd);
        return MPP_ERR_NULL_PTR;
    }
    p = mpp_calloc(MpiEncTestData, 1);
    if (!p) {
        printf("create MpiEncTestData failed\n");
        ret = MPP_ERR_MALLOC;
        goto RET;
    }
    memset(p, 0, sizeof(MpiEncTestData));

    // get paramter from cmd
    p->width        = cmd->width;
    p->height       = cmd->height;
    p->hor_stride   = MPP_ALIGN(cmd->width, 16);
    p->ver_stride   = MPP_ALIGN(cmd->height, 16);
    p->fmt          = cmd->format;
    p->type         = cmd->type;
    if (cmd->type == MPP_VIDEO_CodingMJPEG)
        cmd->num_frames = 1;
    p->num_frames   = cmd->num_frames;

    if (cmd->have_input) {
        p->fp_input = fopen(cmd->file_input, "rb");
        if (NULL == p->fp_input) {
            printf("failed to open input file %s\n", cmd->file_input);
            printf("create default yuv image for test\n");
        }
    }

    if (cmd->have_output) {
        p->fp_output = fopen(cmd->file_output, "w+b");
        if (NULL == p->fp_output) {
            printf("failed to open output file %s\n", cmd->file_output);
            ret = MPP_ERR_OPEN_FILE;
        }
    }

    // update resource parameter
    if (p->fmt <= MPP_FMT_YUV420SP_VU)
        p->frame_size = p->hor_stride * p->ver_stride * 3 / 2;
    else if (p->fmt <= MPP_FMT_YUV422_UYVY) {
        // NOTE: yuyv and uyvy need to double stride
        p->hor_stride *= 2;
        p->frame_size = p->hor_stride * p->ver_stride;
    } else
        p->frame_size = p->hor_stride * p->ver_stride * 4;
    p->packet_size  = p->width * p->height;

RET:
    *data = p;
    return ret;
}
MPP_RET test_ctx_deinit(MpiEncTestData **data)
{
    MpiEncTestData *p = NULL;

    if (!data) {
        printf("invalid input data %p\n", data);
        return MPP_ERR_NULL_PTR;
    }

    p = *data;
    if (p) {
        if (p->fp_input) {
            fclose(p->fp_input);
            p->fp_input = NULL;
        }
        if (p->fp_output) {
            fclose(p->fp_output);
            p->fp_output = NULL;
        }
        MPP_FREE(p);
        *data = NULL;
    }

    return MPP_OK;
}
MPP_RET test_mpp_setup(MpiEncTestData *p)
{
    MPP_RET ret;
    MppApi *mpi;
    MppCtx ctx;
    MppEncCodecCfg *codec_cfg;
    MppEncPrepCfg *prep_cfg;
    MppEncRcCfg *rc_cfg;

    if (NULL == p)
        return MPP_ERR_NULL_PTR;

    mpi = p->mpi;
    ctx = p->ctx;
    codec_cfg = &p->codec_cfg;
    prep_cfg = &p->prep_cfg;
    rc_cfg = &p->rc_cfg;

    /* setup default parameter */
    p->fps = 30;
    p->gop = 60;
    p->bps = p->width * p->height / 8 * p->fps;

    prep_cfg->change        = MPP_ENC_PREP_CFG_CHANGE_INPUT |
                              MPP_ENC_PREP_CFG_CHANGE_ROTATION |
                              MPP_ENC_PREP_CFG_CHANGE_FORMAT;
    prep_cfg->width         = p->width;
    prep_cfg->height        = p->height;
    prep_cfg->hor_stride    = p->hor_stride;
    prep_cfg->ver_stride    = p->ver_stride;
    prep_cfg->format        = p->fmt;
    prep_cfg->rotation      = MPP_ENC_ROT_0;

    ret = mpi->control(ctx, MPP_ENC_SET_PREP_CFG, prep_cfg);
    if (ret) {
        printf("mpi control enc set prep cfg failed ret %d\n", ret);
        goto RET;
    }

    rc_cfg->change  = MPP_ENC_RC_CFG_CHANGE_ALL;
    rc_cfg->rc_mode = MPP_ENC_RC_MODE_CBR;
    rc_cfg->quality = MPP_ENC_RC_QUALITY_MEDIUM;

    if (rc_cfg->rc_mode == MPP_ENC_RC_MODE_CBR) {
        /* constant bitrate has very small bps range of 1/16 bps */
        rc_cfg->bps_target   = p->bps;
        rc_cfg->bps_max      = p->bps * 17 / 16;
        rc_cfg->bps_min      = p->bps * 15 / 16;
    } else if (rc_cfg->rc_mode ==  MPP_ENC_RC_MODE_VBR) {
        if (rc_cfg->quality == MPP_ENC_RC_QUALITY_CQP) {
            /* constant QP does not have bps */
            rc_cfg->bps_target   = -1;
            rc_cfg->bps_max      = -1;
            rc_cfg->bps_min      = -1;
        } else {
            /* variable bitrate has large bps range */
            rc_cfg->bps_target   = p->bps;
            rc_cfg->bps_max      = p->bps * 17 / 16;
            rc_cfg->bps_min      = p->bps * 1 / 16;
        }
    }

    /* fix input / output frame rate */
    rc_cfg->fps_in_flex      = 0;
    rc_cfg->fps_in_num       = p->fps;
    rc_cfg->fps_in_denorm    = 1;
    rc_cfg->fps_out_flex     = 0;
    rc_cfg->fps_out_num      = p->fps;
    rc_cfg->fps_out_denorm   = 1;

    rc_cfg->gop              = p->gop;
    rc_cfg->skip_cnt         = 0;

    //printf("mpi_enc_test bps %d fps %d gop %d\n", rc_cfg->bps_target, rc_cfg->fps_out_num, rc_cfg->gop);

    ret = mpi->control(ctx, MPP_ENC_SET_RC_CFG, rc_cfg);
    if (ret) {
        printf("mpi control enc set rc cfg failed ret %d\n", ret);
        goto RET;
    }

    codec_cfg->coding = p->type;
    switch (codec_cfg->coding) {
    case MPP_VIDEO_CodingAVC : {
        codec_cfg->h264.change = MPP_ENC_H264_CFG_CHANGE_PROFILE |
                                 MPP_ENC_H264_CFG_CHANGE_ENTROPY |
                                 MPP_ENC_H264_CFG_CHANGE_TRANS_8x8;
        /*
         * H.264 profile_idc parameter
         * 66  - Baseline profile
         * 77  - Main profile
         * 100 - High profile
         */
        codec_cfg->h264.profile  = 100;
        /*
         * H.264 level_idc parameter
         * 10 / 11 / 12 / 13    - qcif@15fps / cif@7.5fps / cif@15fps / cif@30fps
         * 20 / 21 / 22         - cif@30fps / half-D1@@25fps / D1@12.5fps
         * 30 / 31 / 32         - D1@25fps / 720p@30fps / 720p@60fps
         * 40 / 41 / 42         - 1080p@30fps / 1080p@30fps / 1080p@60fps
         * 50 / 51 / 52         - 4K@30fps
         */
        codec_cfg->h264.level    = 31;
        codec_cfg->h264.entropy_coding_mode  = 1;
        codec_cfg->h264.cabac_init_idc  = 0;
        codec_cfg->h264.transform8x8_mode = 1;
    } break;
    case MPP_VIDEO_CodingMJPEG : {
        codec_cfg->jpeg.change  = MPP_ENC_JPEG_CFG_CHANGE_QP;
        codec_cfg->jpeg.quant   = 10;
    } break;
    case MPP_VIDEO_CodingVP8 : {
    } break;
    case MPP_VIDEO_CodingHEVC : {
        codec_cfg->h265.change = MPP_ENC_H265_CFG_INTRA_QP_CHANGE;
        codec_cfg->h265.intra_qp = 26;
    } break;
    default : {
        printf("support encoder coding type %d\n", codec_cfg->coding);
    } break;
    }

    ret = mpi->control(ctx, MPP_ENC_SET_CODEC_CFG, codec_cfg);
    if (ret) {
        printf("mpi control enc set codec cfg failed ret %d\n", ret);
        goto RET;
    }

    /* optional */
    p->sei_mode = MPP_ENC_SEI_MODE_ONE_FRAME;

    ret = mpi->control(ctx, MPP_ENC_SET_SEI_CFG, &p->sei_mode);
    if (ret) {
        printf("mpi control enc set sei cfg failed ret %d\n", ret);
        goto RET;
    }

RET:
    return ret;
}
MPP_RET test_mpp_run(MpiEncTestData *p, void *input, void *output, int *out_size)
{
    MPP_RET ret;
    MppApi *mpi;
    MppCtx ctx;

    if (NULL == p)
        return MPP_ERR_NULL_PTR;

    mpi = p->mpi;
    ctx = p->ctx;

    if (p->type == MPP_VIDEO_CodingAVC) {
        MppPacket packet = NULL;
        ret = mpi->control(ctx, MPP_ENC_GET_EXTRA_INFO, &packet);
        if (ret) {
            printf("mpi control enc get extra info failed\n");
            goto RET;
        }

        /* get and write sps/pps for H.264 */
        if (packet) {
            void *ptr   = mpp_packet_get_pos(packet);
            size_t len  = mpp_packet_get_length(packet);

            if (p->fp_output)
                fwrite(ptr, 1, len, p->fp_output);
	    else {
		memcpy(output, ptr, len);
		*out_size = len;
	    }


            packet = NULL;
        }
    }

    while (!p->pkt_eos) {
        MppFrame frame = NULL;
        MppPacket packet = NULL;
        void *buf = mpp_buffer_get_ptr(p->frm_buf);

        if (p->fp_input) {

            /*ret = read_yuv_image(buf, p->fp_input, p->width, p->height,
                                 p->hor_stride, p->ver_stride, p->fmt);*/
            if (ret == MPP_NOK || feof(p->fp_input)) {
                printf("found last frame. feof %d\n", feof(p->fp_input));
                p->frm_eos = 1;
            } else if (ret == MPP_ERR_VALUE)
                goto RET;
        } else {
            /*ret = fill_yuv_image(buf, p->width, p->height, p->hor_stride,
                                 p->ver_stride, p->fmt, p->frame_count);*/

	    memcpy(buf, input, p->frame_size);//hisping
            if (ret)
                goto RET;
        }

        ret = mpp_frame_init(&frame);
        if (ret) {
            printf("mpp_frame_init failed\n");
            goto RET;
        }

        mpp_frame_set_width(frame, p->width);
        mpp_frame_set_height(frame, p->height);
        mpp_frame_set_hor_stride(frame, p->hor_stride);
        mpp_frame_set_ver_stride(frame, p->ver_stride);
        mpp_frame_set_fmt(frame, p->fmt);
        mpp_frame_set_eos(frame, p->frm_eos);

        if (p->fp_input && feof(p->fp_input)){
            mpp_frame_set_buffer(frame, NULL);
        }else{

            mpp_frame_set_buffer(frame, p->frm_buf);
	}
        ret = mpi->encode_put_frame(ctx, frame);
        if (ret) {
            printf("mpp encode put frame failed\n");
            goto RET;
        }

        ret = mpi->encode_get_packet(ctx, &packet);
        if (ret) {
            printf("mpp encode get packet failed\n");
            goto RET;
        }

        if (packet) {
            // write packet to file here
            void *ptr   = mpp_packet_get_pos(packet);
            size_t len  = mpp_packet_get_length(packet);

            p->pkt_eos = mpp_packet_get_eos(packet);

            if (p->fp_output)
                fwrite(ptr, 1, len, p->fp_output);
	    else {

		memcpy(output + (*out_size), ptr, len);
		*out_size += len;
	    }//hisping

            mpp_packet_deinit(&packet);
#ifdef SHOW_LOG
            printf("encoded frame %d size %d\n", p->frame_count, len);
#endif
            p->stream_size += len;
            p->frame_count++;

            if (p->pkt_eos) {
                printf("found last packet\n");
                //mpp_assert(p->frm_eos);
            }
        }

        if (p->num_frames && p->frame_count >= p->num_frames) {
#ifdef SHOW_LOG
            printf("encode max %d frames", p->frame_count);
#endif
            break;
        }
        if (p->frm_eos && p->pkt_eos)
            break;
    }
RET:

    return ret;
}
int mpi_enc_test(MpiEncTestCmd *cmd, void *input, void *output, int *out_size)
{
    MPP_RET ret = MPP_OK;
    MpiEncTestData *p = NULL;

    //printf("mpi_enc_test start\n");

    ret = test_ctx_init(&p, cmd);
    if (ret) {
        printf("test data init failed ret %d\n", ret);
        goto MPP_TEST_OUT;
    }

    ret = mpp_buffer_get(NULL, &p->frm_buf, p->frame_size);
    if (ret) {
        printf("failed to get buffer for input frame ret %d\n", ret);
        goto MPP_TEST_OUT;
    }

    //printf("mpi_enc_test encoder test start w %d h %d type %d\n", p->width, p->height, p->type);

    // encoder demo
    ret = mpp_create(&p->ctx, &p->mpi);
    if (ret) {
        printf("mpp_create failed ret %d\n", ret);
        goto MPP_TEST_OUT;
    }

    ret = mpp_init(p->ctx, MPP_CTX_ENC, p->type);
    if (ret) {
        printf("mpp_init failed ret %d\n", ret);
        goto MPP_TEST_OUT;
    }

    ret = test_mpp_setup(p);
    if (ret) {
        printf("test mpp setup failed ret %d\n", ret);
        goto MPP_TEST_OUT;
    }

    ret = test_mpp_run(p, input, output, out_size);
    if (ret) {
        printf("test mpp run failed ret %d\n", ret);
        goto MPP_TEST_OUT;
    }

    ret = p->mpi->reset(p->ctx);
    if (ret) {
        printf("mpi->reset failed\n");
        goto MPP_TEST_OUT;
    }

MPP_TEST_OUT:
    if (p->ctx) {
        mpp_destroy(p->ctx);
        p->ctx = NULL;
    }

    if (p->frm_buf) {
        mpp_buffer_put(p->frm_buf);
        p->frm_buf = NULL;
    }

    if (MPP_OK == ret)
    ;
        // printf("mpi_enc_test success total frame %d bps %lld\n", p->frame_count, (RK_U64)((p->stream_size * 8 * p->fps) / p->frame_count));
    else
        printf("mpi_enc_test failed ret %d\n", ret);

    test_ctx_deinit(&p);

    return ret;
}
void StreamEncoder::flushInput() {

}

////////////////////////// H264or5Fragmenter implementation ////////////////////////////////

StreamEncoder *StreamEncoder::createNew(UsageEnvironment& env,FramedSource* inputSource,
				unsigned inputBufferMax)
{
  	return new StreamEncoder(env,inputSource,inputBufferMax);
}

StreamEncoder::StreamEncoder(UsageEnvironment& env, FramedSource* inputSource, unsigned inputBufferMax)
  : FramedFilter(env, inputSource),
    fOutputBufferSize(0),fOutputBuffer(NULL),
    fNumValidDataBytes(0),fMaxOfFrameToSend(1),fTotOfFrameToSend(0),
    width(640),height(480),
    pictureSize(0),srcfmt(AV_PIX_FMT_NV12)
{

  	pictureSize = avpicture_get_size(srcfmt, width, height);
	printf("pictureSize=%d\n", pictureSize);
	srcbuf = (unsigned char *)malloc(pictureSize);

}

StreamEncoder::~StreamEncoder() {
  	printf("__StreamEncoder::H264_encode_close__\n");
	free(srcbuf);
}


unsigned StreamEncoder::maxFrameSize() const {
  	return 80 * 1024;
}


void StreamEncoder::registerInputInterest(void) {
  	fOutputBufferSize=fMaxSize;
  	fOutputBuffer=fTo;
  	fTotOfFrameToSend=0;
  	fNumValidDataBytes=0;
}

void StreamEncoder::reset()
{
  	fNumValidDataBytes=0;
  	fOutputBufferSize=0;
  	fOutputBuffer=NULL;
  	fTotOfFrameToSend=0;
}



void StreamEncoder::copy_to_outputbuffer(void)
{
      	memmove(fOutputBuffer, dstbuf, dstsize);
      	fNumValidDataBytes += dstsize;
      	fOutputBuffer += dstsize;
      	fTotOfFrameToSend++;
      	//printf("__fNumValidDataBytes:%d\n",fNumValidDataBytes);
      	//printf("dstsize:%d\n",dstsize);
	if(dstbuf != NULL)
		free(dstbuf);
}


void StreamEncoder::encoder_to_h264(void)
{
	MpiEncTestCmd  cmd_ctx;
	MpiEncTestCmd* cmd = &cmd_ctx;
	memset((void*)cmd, 0, sizeof(*cmd));

	cmd->width = width;
	cmd->height = height;
	cmd->format = MPP_FMT_YUV420SP;
	cmd->type = MPP_VIDEO_CodingAVC;
	cmd->num_frames = 1;


	dstbuf = (uint8_t *)malloc(pictureSize);
	dstsize = 0;
	mpi_enc_test(cmd, srcbuf, dstbuf, &dstsize);
}




void StreamEncoder::continueReadProcessing1(unsigned frameSize,unsigned numTruncatedBytes,
					  struct timeval presentationTime,
					  unsigned durationInMicroseconds){

  	fNumTruncatedBytes = numTruncatedBytes;
  	fPresentationTime = presentationTime;
  	fDurationInMicroseconds = durationInMicroseconds;

  	encoder_to_h264();

      	if(fNumValidDataBytes+dstsize<fOutputBufferSize && fTotOfFrameToSend<fMaxOfFrameToSend)
      	{
        	copy_to_outputbuffer();		
      	}

	fFrameSize = fNumValidDataBytes;
	gettimeofday(&fPresentationTime, NULL);
	reset();
      	FramedSource::afterGetting(this); 	
}


void StreamEncoder::continueReadProcessing(void* clientData,
            				unsigned frameSize,unsigned numTruncatedBytes,
					struct timeval presentationTime,
					unsigned durationInMicroseconds) {
  	StreamEncoder* encoder = (StreamEncoder*)clientData;
  	encoder->continueReadProcessing1(frameSize,numTruncatedBytes,
					  presentationTime,
					  durationInMicroseconds);

}

void StreamEncoder::readAndProcessing(void)
{
	fInputSource->getNextFrame(srcbuf,pictureSize,
	StreamEncoder::continueReadProcessing, this,
      	FramedSource::handleClosure, this); 

}

void StreamEncoder::doGetNextFrame() {

	long begin = get_time();
  	registerInputInterest();
  	readAndProcessing();
	long end = get_time();

}

void StreamEncoder::doStopGettingFrames()
{

}
