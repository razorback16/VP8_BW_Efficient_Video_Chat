#include "rgb_to_yuv420.h"

using namespace std;
using namespace cv;


inline void rgb_to_yuv(unsigned char   b, unsigned char   g, unsigned char   r,
                       unsigned char & y, unsigned char & u, unsigned char & v)
{
    float yf, uf, vf;

    //Y = R * 0.299 + G * 0.587 + B * 0.114;
    //U = R * -0.169 + G * -0.332 + B * 0.500 + 128.0;
    //V = R * 0.500 + G * -0.419 + B * -0.0813 + 128.0;

    yf =    0.299f * static_cast<float>(r) +
            0.587f * static_cast<float>(g) +
            0.114f * static_cast<float>(b);
    yf = (yf > 255.0f) ? 255.0f: yf;
    yf = (yf < 0.0f) ? 0.0f: yf;
    y = static_cast<unsigned char>(yf);


    uf =   -0.169f * static_cast<float>(r) -
            0.332f * static_cast<float>(g) +
            0.500f * static_cast<float>(b) + 128.0;
    uf = (uf > 255.0f) ? 255.0f: uf;
    uf = (uf < 0.0f) ? 0.0f: uf;
    u = static_cast<unsigned char>(uf);


    vf =    0.500f * static_cast<float>(r) -
            0.419f * static_cast<float>(g) -
            0.081f * static_cast<float>(b) + 128.0;
    vf = (vf > 255.0f) ? 255.0f: vf;
    vf = (vf < 0.0f) ? 0.0f: vf;
    v = static_cast<unsigned char>(vf);

}

void RGB_to_YUV420(const char * rgb, vpx_image_t *yuv420, int width, int height)
{
    unsigned char * y_pixel = yuv420->planes[0];
    unsigned char * u_pixel = yuv420->planes[1];
    unsigned char * v_pixel = yuv420->planes[2];

    unsigned char * U_tmp = new unsigned char [width * height];
    unsigned char * V_tmp = new unsigned char [width * height];

    int index = 0;
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            rgb_to_yuv(rgb[3 * (y * width + x) + 0], rgb[3 * (y * width + x) + 1], rgb[3 * (y * width + x) + 2], y_pixel[index], U_tmp[index], V_tmp[index]);
            index++;
        }
    }

    index = 0;
    for (int y = 0; y < height; y+=2)
    {
        for (int x = 0; x < width; x+=2)
        {
            u_pixel[index] = U_tmp[y * width + x];
            v_pixel[index] = V_tmp[y * width + x];
            index++;
        }
    }

    delete [] U_tmp;
    delete [] V_tmp;
}


// void YUV420_To_IplImage(vpx_image_t *pYUV420, IplImage * pRGBimg, FILE *outfile)
// {
//     if (!pRGBimg || !pYUV420)
//     {
//         return;
//     }

//     IplImage *yuvimage,*yimg,*uimg,*vimg,*uuimg,*vvimg;

//     int nWidth = pYUV420->d_w;
//     int nHeight = pYUV420->d_h;

//     printf("w=%d,h=%d\n", nWidth, nHeight);

//     //cvNamedWindow( "result2", 2 );


//     //yuvimage = cvCreateImage(cvSize(nWidth, nHeight),IPL_DEPTH_8U,3);

//     //yimg = cvCreateImage(cvSize(nWidth, nHeight),IPL_DEPTH_8U,1);
//     //uimg = cvCreateImageHeader(cvSize(nWidth/2, nHeight/2),IPL_DEPTH_8U,1);
//     //vimg = cvCreateImageHeader(cvSize(nWidth/2, nHeight/2),IPL_DEPTH_8U,1);

//     //uuimg = cvCreateImage(cvSize(nWidth, nHeight),IPL_DEPTH_8U,1);
//     //vvimg = cvCreateImage(cvSize(nWidth, nHeight),IPL_DEPTH_8U,1);

//     //cvSetData(pRGBimg,pYUV420->planes[0], nWidth);
//     //cvSetData(uimg,pYUV420->planes[1], nWidth/2);
//     //cvSetData(vimg,pYUV420->planes[2], nWidth/2);
//     vpx_img_write(pYUV420, outfile);

//     memcpy(pRGBimg->imageData,pYUV420->planes[0],nWidth*nHeight);

//     //cvShowImage( "result2", pRGBimg );

//     //cvResize(uimg,uuimg,CV_INTER_LINEAR);
//     //cvResize(vimg,vvimg,CV_INTER_LINEAR);

//     //cvMerge(yimg,yimg,yimg,NULL,pRGBimg);
//     //cvCvtColor(yuvimage,pRGBimg,CV_YCrCb2RGB);

//     //cvReleaseImage(&uuimg);
//     //cvReleaseImage(&vvimg);
//     //cvReleaseImage(&yimg);
//     //cvReleaseImageHeader(&uimg);
//     //cvReleaseImageHeader(&vimg);

//     //cvReleaseImage(&yuvimage);

//     return;
// }

inline int clamp8(int v)
{
    return std::min(std::max(v, 0), 255);
} 

void YUV420_To_IplImage(vpx_image_t* img, IplImage * pRGBimg)
{
    char *data = pRGBimg->imageData;

    uint8_t *yPlane = img->planes[VPX_PLANE_Y];
    uint8_t *uPlane = img->planes[VPX_PLANE_U];
    uint8_t *vPlane = img->planes[VPX_PLANE_V];

    int i = 0;
    for (unsigned int imgY = 0; imgY < img->d_h; imgY++) {
        for (unsigned int imgX = 0; imgX < img->d_w; imgX++) {
            int y = yPlane[imgY * img->stride[VPX_PLANE_Y] + imgX];
            int u = uPlane[(imgY / 2) * img->stride[VPX_PLANE_U] + (imgX / 2)];
            int v = vPlane[(imgY / 2) * img->stride[VPX_PLANE_V] + (imgX / 2)];

            int c = y - 16;
            int d = (u - 128);
            int e = (v - 128);

            // TODO: adjust colors ?

            int b = clamp8((298 * c           + 409 * e + 128) >> 8);
            int g = clamp8((298 * c - 100 * d - 208 * e + 128) >> 8);
            int r = clamp8((298 * c + 516 * d           + 128) >> 8);

            // TODO: cast instead of clamp8

            data[i + 0] = static_cast<uint8_t>(r);
            data[i + 1] = static_cast<uint8_t>(g);
            data[i + 2] = static_cast<uint8_t>(b);

            i += 3;
        }
    }
    return;
}


// void VPXImageToRGB24(vpx_image_t* img, IplImage * pRGBimg)
// {
//    const unsigned int rgbBufferSize = pImage->d_w * pImage->d_h * 3;

//    mpRGBBuffer - allocate your raw RGB buffer...

//     const IppiSize  sz          = { pImage->d_w, pImage->d_h };
//     const Ipp8u*    src[3]      = { pImage->planes[PLANE_Y],     pImage->planes[PLANE_U],     pImage->planes[PLANE_V]       };
//     int             srcStep[3]  = { pImage->stride[VPX_PLANE_Y], pImage->stride[VPX_PLANE_U], pImage->stride[VPX_PLANE_V]   };

//     if (isUsingBGR) ippiYCbCr420ToBGR_8u_P3C3R(src, srcStep, pDest, pImage->d_w * 3, sz);
//     else            ippiYCbCr420ToRGB_8u_P3C3R(src, srcStep, pDest, pImage->d_w * 3, sz);
// }

void IplImage_To_YUV420(IplImage * pRGBimg, vpx_image_t *pYUV420)
{
    if (!pRGBimg || !pYUV420)
    {
        return;
    }

    IplImage *yuvimage,*yimg,*uimg,*vimg,*uuimg,*vvimg;

    int nWidth = pRGBimg->width;
    int nHeight = pRGBimg->height;

    yuvimage = cvCreateImage(cvSize(nWidth, nHeight),IPL_DEPTH_8U,3);

    yimg = cvCreateImage(cvSize(nWidth, nHeight),IPL_DEPTH_8U,1);
    uimg = cvCreateImage(cvSize(nWidth, nHeight),IPL_DEPTH_8U,1);
    vimg = cvCreateImage(cvSize(nWidth, nHeight),IPL_DEPTH_8U,1);

    uuimg = cvCreateImage(cvSize(nWidth/2, nHeight/2),IPL_DEPTH_8U,1);
    vvimg = cvCreateImage(cvSize(nWidth/2, nHeight/2),IPL_DEPTH_8U,1);

    cvCvtColor(pRGBimg,yuvimage,CV_RGB2YCrCb);

    cvSplit(yuvimage, yimg, uimg, vimg, NULL);

    cvResize(uimg,uuimg,CV_INTER_LINEAR);
    cvResize(vimg,vvimg,CV_INTER_LINEAR);

    memcpy(pYUV420->planes[0],yimg->imageData,nWidth*nHeight);
    memcpy(pYUV420->planes[1],uuimg->imageData,(nWidth/2)*(nHeight/2));
    memcpy(pYUV420->planes[2],vvimg->imageData,(nWidth/2)*(nHeight/2));

    cvReleaseImage(&uuimg);
    cvReleaseImage(&vvimg);
    cvReleaseImage(&yimg);
    cvReleaseImage(&uimg);
    cvReleaseImage(&vimg);
    cvReleaseImage(&yuvimage);

    return;
}

