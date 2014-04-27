/*
 *  Copyright (c) 2010 The WebM project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */


// VP8 Set Active and ROI Maps
// ===========================
//
// This is an example demonstrating how to control the VP8 encoder's
// ROI and Active maps.
//
// ROI (Reigon of Interest) maps are a way for the application to assign
// each macroblock in the image to a region, and then set quantizer and
// filtering parameters on that image.
//
// Active maps are a way for the application to specify on a
// macroblock-by-macroblock basis whether there is any activity in that
// macroblock.
//
//
// Configuration
// -------------
// An ROI map is set on frame 22. If the width of the image in macroblocks
// is evenly divisble by 4, then the output will appear to have distinct
// columns, where the quantizer, loopfilter, and static threshold differ
// from column to column.
//
// An active map is set on frame 33. If the width of the image in macroblocks
// is evenly divisble by 4, then the output will appear to have distinct
// columns, where one column will have motion and the next will not.
//
// The active map is cleared on frame 44.
//
// Observing The Effects
// ---------------------
// Use the `simple_decoder` example to decode this sample, and observe
// the change in the image at frames 22, 33, and 44.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cctype>
#include <iostream>
#include <iterator>

using namespace std;
using namespace cv;

typedef struct sockaddr SA;
 	
#define MAXLINE 8192 /* max text line length */

#define VPX_CODEC_DISABLE_COMPAT 1
#include "vpx/vp8cx.h"
#include "vpx/vpx_encoder.h"

#include "../libvpx/tools_common.h"
#include "../libvpx/video_writer.h"

#include "rgb_to_yuv420.h"

string cascadeName = "/home/subhagato/Codes/opencv-2.4.9/data/haarcascades/haarcascade_frontalface_alt.xml";

int costThresh1,costThresh2;

//int vpx_img_read_cv_img(vpx_image_t *img, IplImage *frame);

int open_clientfd(char *hostname, int port) 
{
    int clientfd;
    struct hostent *hp;
    struct sockaddr_in serveraddr;

    if ((clientfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	return -1; /* Check errno for cause of error */

    /* Fill in the server's IP address and port */
    if ((hp = gethostbyname(hostname)) == NULL)
	return -2; /* Check h_errno for cause of error */
    bzero((char *) &serveraddr, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    bcopy((char *)hp->h_addr_list[0], 
	  (char *)&serveraddr.sin_addr.s_addr, hp->h_length);
    serveraddr.sin_port = htons(port);

    /* Establish a connection with the server */
    if (connect(clientfd, (SA *) &serveraddr, sizeof(serveraddr)) < 0)
	return -1;
    return clientfd;
}

static const char *exec_name;

void usage_exit() {
  fprintf(stderr, "Usage: %s codec ip_address <file>\n",
          exec_name);
  exit(EXIT_FAILURE);
}

int determineRegion(vector<Rect> faces, double scale, int x, int y);

void detectDrawAndSetMaps(const vpx_codec_enc_cfg_t *cfg,
                        vpx_codec_ctx_t *codec, Mat& img, CascadeClassifier& cascade,
                            double scale, FILE *fp);


static void encode_frame(vpx_codec_ctx_t *codec,
                         vpx_image_t *img,
                         int frame_index,
                         VpxVideoWriter *writer) {
  vpx_codec_iter_t iter = NULL;
  const vpx_codec_cx_pkt_t *pkt = NULL;
  const vpx_codec_err_t res = vpx_codec_encode(codec, img, frame_index, 1, 0,
                                               VPX_DL_GOOD_QUALITY);
  if (res != VPX_CODEC_OK)
    die_codec(codec, "Failed to encode frame");

  while ((pkt = vpx_codec_get_cx_data(codec, &iter)) != NULL) {
    if (pkt->kind == VPX_CODEC_CX_FRAME_PKT) {
      const int keyframe = (pkt->data.frame.flags & VPX_FRAME_IS_KEY) != 0;
      if (!vpx_video_writer_write_frame(writer,
                                        (const uint8_t *)pkt->data.frame.buf,
                                        pkt->data.frame.sz,
                                        pkt->data.frame.pts)) {
        die_codec(codec, "Failed to write compressed frame");
      }

      printf(keyframe ? "K" : ".");
      fflush(stdout);
    }
  }
}

int main(int argc, char **argv) {

  vpx_codec_ctx_t codec = {0};
  vpx_codec_enc_cfg_t cfg = {0};
  int frame_count = 0;
  vpx_image_t raw;
  vpx_codec_err_t res;
  VpxVideoInfo info = {0};
  VpxVideoWriter *writer = NULL;
  const VpxInterface *encoder = NULL;
  const int fps = 2;        // TODO(dkovalev) add command line argument
  const double bits_per_pixel_per_frame = 0.067;
  int fd;
  exec_name = argv[0];
  FILE *fp=NULL;

  IplImage* iplImg;

  CvCapture *capture = 0;
  Mat frame, frameCopy, image;

  CascadeClassifier cascade;
  double scale = 3;

  if (argc > 4 || argc<3)
    die("Invalid number of arguments");

  encoder = get_vpx_encoder_by_name(argv[1]);
  if (!encoder)
  die("Unsupported codec.");

  if( !cascade.load( cascadeName ) )
  {
    cerr << "ERROR: Could not load classifier cascade" << endl;
    return -1;
  }

  capture = cvCaptureFromCAM( 0 );

  if(!capture) cout << "Capture from CAM 0 didn't work" << endl;

  iplImg = cvQueryFrame( capture );

  frame = iplImg;

  if(frame.empty())
    die("Unable to capture frame");

  info.codec_fourcc = encoder->fourcc;
  info.frame_width = iplImg->width;
  info.frame_height = iplImg->height;
  info.time_base.numerator = 1;
  info.time_base.denominator = fps;

  costThresh1 =  (int)(sqrt((double)((iplImg->width)*(iplImg->width)+(iplImg->height)*(iplImg->height)))/100);
  costThresh2 =  (int)(sqrt((double)((iplImg->width)*(iplImg->width)+(iplImg->height)*(iplImg->height)))/44);

  fd = open_clientfd(argv[2], 15213); 

  if(argc==4)
  {
    if((fp = fopen(argv[3],"w"))<=0)
      die("Unable to open file %s for writing!", argv[3]);
  }
    
  

 if (info.frame_width <= 0 ||
      info.frame_height <= 0 ||
      (info.frame_width % 2) != 0 ||
      (info.frame_height % 2) != 0) {
    die("Invalid frame size: %dx%d", info.frame_width, info.frame_height);
  }

  if (!vpx_img_alloc(&raw, VPX_IMG_FMT_I420, info.frame_width,
                                             info.frame_height, 1)) {
    die("Failed to allocate image.");
  }

  printf("Using %s\n", vpx_codec_iface_name(encoder->interface()));

  res = vpx_codec_enc_config_default(encoder->interface(), &cfg, 0);
  if (res)
    die_codec(&codec, "Failed to get default codec config.");

  cfg.g_w = info.frame_width;
  cfg.g_h = info.frame_height;
  cfg.g_timebase.num = info.time_base.numerator;
  cfg.g_timebase.den = info.time_base.denominator;
  cfg.rc_target_bitrate = (unsigned int)(bits_per_pixel_per_frame * cfg.g_w *
                                         cfg.g_h * fps / 1000);
  cfg.g_lag_in_frames = 0;

  writer = vpx_video_writer_open_c(fd, kContainerIVF, &info);
  if (!writer)
    die("Failed to open %s for writing.", argv[2]);
//-------------------------------------------------------------------------------------
  //writer = vpx_video_writer_open(fd, kContainerIVF, &info);
  //if (!writer)
    //die("Failed to open %s for writing.", argv[5]);
//-------------------------------------------------------------------------------------

  if(capture)
  {

    cvNamedWindow( "result", 1 );

    if (vpx_codec_enc_init(&codec, encoder->interface(), &cfg, 0))
      die_codec(&codec, "Failed to initialize encoder");

    while (1) {

        IplImage* iplImg = cvQueryFrame( capture );
        frame = iplImg;

        if( frame.empty() )
            break;

        ++frame_count;


        if( iplImg->origin == IPL_ORIGIN_TL )
            frame.copyTo( frameCopy );
        else
            flip( frame, frameCopy, 0 );

        if(fp)
          fprintf(fp,"frame=%d\n",frame_count);

        detectDrawAndSetMaps(&cfg, &codec, frameCopy, cascade, scale, fp);
        
        //vpx_img_read_cv_img(&raw, iplImg);
        IplImage_To_YUV420(iplImg, &raw);

        encode_frame(&codec, &raw, frame_count, writer);

//        cvReleaseImage(&iplImg);

        if( waitKey( 10 ) >= 0 )
            goto _cleanup_;
      }

      waitKey(0);

_cleanup_:
    
    cvReleaseCapture( &capture );

  if(argc==4)
    fclose(fp);

  encode_frame(&codec, NULL, -1, writer);
  printf("\n");
  printf("Processed %d frames.\n", frame_count);

  vpx_img_free(&raw);
  if (vpx_codec_destroy(&codec))
    die_codec(&codec, "Failed to destroy codec.");

  vpx_video_writer_close(writer);

  cvDestroyWindow("result");
  
  }

  return EXIT_SUCCESS;
}


// int vpx_img_read_cv_img(vpx_image_t *img, IplImage *frame) {

//   //RGB_to_YUV420(frame->imageData, img, frame->width, frame->height);

//   IplImage_To_YUV420(frame, img);

//   return 1;
// }




void detectDrawAndSetMaps(const vpx_codec_enc_cfg_t *cfg,
                        vpx_codec_ctx_t *codec, Mat& img, CascadeClassifier& cascade,
                            double scale, FILE *fp)
{
    unsigned int i = 0, j;
    double t = 0;
    vector<Rect> faces;
    Mat gray, smallImg( cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1 );
    vpx_roi_map_t roi = {0};

    const static Scalar colors[] =  { CV_RGB(0,0,255),
                                      CV_RGB(0,128,255),
                                      CV_RGB(0,255,255),
                                      CV_RGB(0,255,0),
                                      CV_RGB(255,128,0),
                                      CV_RGB(255,255,0),
                                      CV_RGB(255,0,0),
                                      CV_RGB(255,0,255)} ;

    roi.rows = cfg->g_h / 16;
    roi.cols = cfg->g_w / 16;

    roi.delta_q[0] = -63;                                           //
    roi.delta_q[1] = -32;                                          //
    roi.delta_q[2] = +10;                                          //
    roi.delta_q[3] = +63;                                          //
                                                              //
    roi.delta_lf[0] = 0;                                          //
    roi.delta_lf[1] = 0;                                          //
    roi.delta_lf[2] = 0;                                          //
    roi.delta_lf[3] = 0;                                          //
                                                              //
    roi.static_threshold[0] =     0;                              //
    roi.static_threshold[1] =   100;                              //
    roi.static_threshold[2] =  1000;                              //
    roi.static_threshold[3] =  1500;                              //

    roi.roi_map = (uint8_t *)malloc(roi.rows * roi.cols);

    cvtColor( img, gray, CV_BGR2GRAY );
    resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
    equalizeHist( smallImg, smallImg );

    t = (double)cvGetTickCount();
    cascade.detectMultiScale( smallImg, faces,
        1.1, 2, 0
        //|CV_HAAR_FIND_BIGGEST_OBJECT
        //|CV_HAAR_DO_ROUGH_SEARCH
        |CV_HAAR_SCALE_IMAGE
        ,
        Size(30, 30) );

    t = (double)cvGetTickCount() - t;
    printf( "detection time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );
    

    for( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
    {
        // Point center;
        Scalar color = colors[i%8];

        if(fp)
          fprintf(fp,"%d:x=%lf,y=%lf,w=%lf,h=%lf\n",i+1,r->x*scale/16,r->y*scale/16,(r->width*scale)/16,(r->height*scale)/16);

        rectangle( img, cvPoint(cvRound(r->x*scale), cvRound(r->y*scale)),
                       cvPoint(cvRound((r->x + r->width-1)*scale), cvRound((r->y + r->height-1)*scale)),
                       color, 3, 8, 0);

    }

    for(i=0;i<roi.rows;i++)
    {
        for(j=0;j<roi.cols;j++)
        {
          roi.roi_map[i*roi.cols+j] = determineRegion(faces, scale, j, i);
          if(fp)
            fprintf(fp,"%d ",roi.roi_map[i*roi.cols+j]);
        }
        if(fp)
          fprintf(fp,"\n");
    }

    if (vpx_codec_control(codec, VP8E_SET_ROI_MAP, &roi))
    die_codec(codec, "Failed to set ROI map");

    free(roi.roi_map);

    cv::imshow( "result", img );
}

int determineRegion(vector<Rect> faces, double scale, int x, int y)
{
    int i=0;
    int topX,topY,botX,botY,cenX,cenY;
    int minCost=0x7FFFFFFF, cost=0;


    for( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
    {
        topX = r->x*scale/16;
        topY = r->y*scale/16;
        botX = ((r->x + r->width)*scale)/16;
        botY = ((r->y + r->height)*scale)/16;
        cenX = (topX+botX)/2;
        cenY = (topY+botY)/2;

        if((x>=topX && x<=botX) && (y>=topY && y<=botY))
          return 0;
        else
        {
          cost = sqrt((double)((cenX-x)*(cenX-x)+(cenY-y)*(cenY-y)));
          if(cost<minCost)
            minCost = cost;
        }
          
    }

    if(minCost==0x7FFFFFFF)
      return 2; // returns 1, if no faces found
    
    if(minCost<costThresh1)
      return 1;   
    else if(minCost<costThresh2)
      return 2;
    else
      return 3;



}