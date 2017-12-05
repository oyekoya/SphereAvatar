/*
 *      Kinect.h
 *
 *      Fabrizio Pece <f.pece@cs.ucl.ac.uk>
 *		University College London
 *		
 *		2011
 *
 *		HEADER - Camera utilities - Kinect camera class built on top of OpenNI SDK
 */

#ifndef KINECT_H_
#define KINECT_H_

#define NOMINMAX


#include <windows.h>

typedef unsigned char     uint8_t;
typedef unsigned short    uint16_t;
typedef unsigned int      uint32_t;
typedef unsigned long int uint64_t;
typedef int      int32_t;
typedef long int int64_t;

#include "Camera.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <XnOS.h>
#include <XnCppWrapper.h>


#include "pcl/io/pcd_io.h"
#include "pcl/common/common_headers.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>




using namespace xn;

#define SAMPLE_XML_PATH "SamplesConfig.xml"
#define MAX_DEPTH 10000


//Representation of an RGB pixel in the 3D space
typedef struct RGBDpixel
{
	// color
	float	r; 
	float	g; 
	float	b; 
	
	// 3d coordinates in mt
	float	x; 
	float	y; 
	float	z;

	float	d;

} RGBDpixel;

typedef XnVector3D XnPoint3D;

class Kinect:public Camera{

public:
	Kinect();
	~Kinect();

	bool start();
	bool stop();

	const unsigned char* grabRGB();
	const unsigned short* grabDepth();

	DepthGenerator getDepthGenerator();
	ImageGenerator getImageGenerator();
	XnDepthPixel getMaxDepth();

	void alignViewPoint();
	void resetViewPoint();

	
	RGBDpixel** extractRGBDpoints(unsigned char* rgb, unsigned short* depth, int&); // get the RGBd pixels built from the input depth and rgb
	RGBDpixel** extractRGBDpointsIntoPCL(unsigned char* rgb, unsigned short* depth, int&, pcl::PointCloud<pcl::PointXYZRGB>&); 
	void deleteRGBDpoints(RGBDpixel**);
	void convertToWorld(float, float, float, float&, float&, float&);

	XnPoint3D* projective2meters(unsigned short* pDepth); // returns an array of points expressed in mt. If you want to add colourfrom an rgb image, p[0] -> rgb[0],rgb[1],rgb[2];  p[1] -> rgb[3],rgb[4],rgb[5]. A full example of how to do this is at the end of this function in the Kinect.cpp file
	RGBDpixel RGBDpointAt(int i, int j, std::vector<RGBDpixel> buf); // i= row, j=col
	float* projective2metersImage(unsigned short* pDepth); // returns an image (floats) of depths expressed in mt.

	void savePointCloud(char* filename); //saves point cloud. Points are in mm!
	void savePointCloud(RGBDpixel** rgbd,char* filename);
	bool saveImage(char* filename);

	//Eigen::Matrix3f getIntrinsicEigen();
	void getIntrinsicDouble(double*);

private:
	float g_pDepthHist[MAX_DEPTH];
	XnRGB24Pixel* g_pTexMap;
	unsigned int g_nTexMapX;
	unsigned int g_nTexMapY;
	Context g_context;
	DepthGenerator g_depth;
	ImageGenerator g_image;
	DepthMetaData g_depthMD;
	ImageMetaData g_imageMD;
	XnPoint3D focal_length; //calibration params retrieved from http://www.vision.caltech.edu/bouguetj/calib_doc/
	XnPoint3D principal_point;
	std::vector<float> distortion_coeff; //aka, kc coefficient
	//Eigen::Matrix3f intrinsic;

};

#endif //KINECT_H
