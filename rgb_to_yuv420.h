#ifndef __RGB_to_YUV420_H__
#define __RGB_to_YUV420_H__

#include "vpx/vp8cx.h"
#include "vpx/vpx_encoder.h"

#include "../libvpx/tools_common.h"
#include "../libvpx/video_writer.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/imgproc/imgproc_c.h>

void RGB_to_YUV420(const char * rgb, vpx_image_t *yuv420, int width, int height);
void YUV420_To_IplImage(vpx_image_t *pYUV420, IplImage * pRGBimg);
void IplImage_To_YUV420(IplImage * pRGBimg, vpx_image_t *pYUV420);


#endif /* __RGB_to_YUV420_H__ */