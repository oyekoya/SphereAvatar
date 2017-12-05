/*
 *      Kinect.h
 *
 *      Fabrizio Pece <f.pece@cs.ucl.ac.uk>
 *		University College London
 *		
 *		2011
 *
 *		Camera utilities - Kinect camera class built on top of OpenNI SDK
 */


#include "Kinect.h"

//#include "pcl/pcl_base.h"


Kinect::Kinect(){

	setState(false);
	g_nTexMapX = 0;
	g_nTexMapY = 0;
	g_pTexMap = NULL;
	width = 640;
	height = 480;
	focal_length.X = 535.25872f;		focal_length.Y = 536.52895f;		focal_length.Z = 0.0f; //as retrieved from matlab toolbox_calib 
	principal_point.X = 295.30063f;		principal_point.Y = 252.78329f;		principal_point.Z = 0.0f; //as retrieved from matlab toolbox_calib 
	//kc as retrieved from matlab toolbox_calib 
	distortion_coeff.resize(5);
	distortion_coeff[0] = 0.19396f;
	distortion_coeff[1] = -0.28051f;
	distortion_coeff[2] = -0.00003f;
	distortion_coeff[3] = -0.00902f;
	distortion_coeff[4] = 0.00000f;
	//build intrinsic matrix; [fx,0,cx; 0,fy,cy; 0,0,1] 
	/*
	intrinsic = Eigen::Matrix3f::Zero();
	intrinsic(0,0) = focal_length.X;	intrinsic(0,1) = 0.0f;				intrinsic(0,2) = principal_point.X;
	intrinsic(1,0) = 0.0f;				intrinsic(1,1) = focal_length.Y;	intrinsic(1,2) = principal_point.Y;
	intrinsic(2,0) = 0.0f;				intrinsic(2,1) = 0.0f;				intrinsic(2,2) = 1.0f;
	*/
}

Kinect::~Kinect(){

}

/*
Eigen::Matrix3f Kinect::getIntrinsicEigen(){

	return intrinsic;

}
*/ 

void Kinect::getIntrinsicDouble(double* M){


	M[0] = focal_length.X;		M[1] = 0.0f;				M[2] = principal_point.X;
	M[3] = 0.0f;				M[4] = focal_length.Y;		M[5] = principal_point.Y;
	M[6] = 0.0f;				M[7] = 0.0f;				M[8] = 1.0f; 

}

bool Kinect::start(){

	XnStatus rc;
	EnumerationErrors errors;
	rc = g_context.InitFromXmlFile(SAMPLE_XML_PATH, &errors);
	if (rc == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return false;
	}
	else if (rc != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(rc));
		return false;
	}

	rc = g_context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
	rc = g_context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);

	g_depth.GetMetaData(g_depthMD);
	g_image.GetMetaData(g_imageMD);

	// Hybrid mode isn't supported in this sample
	if (g_imageMD.FullXRes() != g_depthMD.FullXRes() || g_imageMD.FullYRes() != g_depthMD.FullYRes())
	{
		printf ("The device depth and image resolution must be equal!\n");
		return false;
	}

	// RGB is the only image format supported.
	if (g_imageMD.PixelFormat() != XN_PIXEL_FORMAT_RGB24)
	{
		printf("The device image format must be RGB24\n");
		return false;
	}

	// Texture map init
	g_nTexMapX = (((unsigned short)(g_depthMD.FullXRes()-1) / 512) + 1) * 512;
	g_nTexMapY = (((unsigned short)(g_depthMD.FullYRes()-1) / 512) + 1) * 512;
	g_pTexMap = (XnRGB24Pixel*)malloc(g_nTexMapX * g_nTexMapY * sizeof(XnRGB24Pixel));

	setState(true);

	return true;

}

const unsigned char* Kinect::grabRGB() {
	
	XnStatus rc = XN_STATUS_OK;
	const XnUInt8* pImage;
	// Read a new frame
	rc = g_context.WaitAnyUpdateAll();
	if (rc != XN_STATUS_OK)
	{
		printf("Read failed: %s\n", xnGetStatusString(rc));
		return pImage;
	}

	g_image.GetMetaData(g_imageMD);

	pImage = g_imageMD.Data();

	return (const unsigned char*)pImage;

}

const unsigned short* Kinect::grabDepth(){

	XnStatus rc = XN_STATUS_OK;
	const XnDepthPixel* pDepth;
	// Read a new frame
	rc = g_context.WaitAnyUpdateAll();
	if (rc != XN_STATUS_OK)
	{
		printf("Read failed: %s\n", xnGetStatusString(rc));
		return pDepth;
	}

	g_depth.GetMetaData(g_depthMD);
	pDepth = g_depthMD.Data();

	return (unsigned short*)pDepth;

}


bool Kinect::stop(){

	return false;

}

void Kinect::alignViewPoint(){

	//align the two sensor viewpoints
	g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);

}

void Kinect::resetViewPoint(){

	//reset the sensor viewpoint
	g_depth.GetAlternativeViewPointCap().ResetViewPoint();

}

DepthGenerator Kinect::getDepthGenerator(){

	return g_depth;

}

ImageGenerator Kinect::getImageGenerator(){

	return g_image;
}

XnDepthPixel Kinect::getMaxDepth(){

	return g_depth.GetDeviceMaxDepth();

}

//filename must end in ply
void Kinect::savePointCloud(char* filename){

	char* ofilename = filename;

	char str[100];
	int s=0;
	while(*filename != NULL){

		str[s] = *filename++;
		s++;

	}

    char *test[10];
    test[0] = strtok(str, "."); // Splits spaces between words in str
    test[1] = strtok (NULL, ".");

	if(strcmp(test[1],"ply")!= 0){

		fprintf(stderr,"Specified wrong format..appending .ply to the filename\n");
		char new_filename[80];
		strcpy(new_filename,test[0]);
		strcat(new_filename,".ply");
		ofilename = new_filename;
		fprintf(stderr,"New filename is %s\n",ofilename);
	}



	unsigned int nNumberOfPoints = 0;

	//align the two sensor viewpoints
	g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);

	//grab depth and rgb
	const XnDepthPixel* pDepth = grabDepth();
	const XnUInt8* pImage = grabRGB();

	//get the total number of points that have to be dumped in the PLY
	for (XnUInt y = 0; y < g_depthMD.YRes(); ++y)
	{
		for (XnUInt x = 0; x < g_depthMD.XRes(); ++x, ++pDepth)
		{
			if (*pDepth!=0)
			{
				nNumberOfPoints++; 
			}
		}
	}

	FILE* ply = fopen(ofilename,"w");
	/*write header*/
	fprintf(ply,"ply\n");
	fprintf(ply,"format ascii 1.0\n");
	fprintf(ply,"element vertex %d\n",nNumberOfPoints);
	fprintf(ply,"property float x\n");
	fprintf(ply,"property float y\n");
	fprintf(ply,"property float z\n");
	fprintf(ply,"property uchar diffuse_red\n");
	fprintf(ply,"property uchar diffuse_green\n");
	fprintf(ply,"property uchar diffuse_blue\n");
	fprintf(ply," end_header\n");

	//RGB
	const XnRGB24Pixel* pImageRow = g_imageMD.RGB24Data();
	XnRGB24Pixel* pTexRow = g_pTexMap + g_imageMD.YOffset() * g_nTexMapX;

	for (XnUInt y = 0; y < g_imageMD.YRes(); ++y){
		const XnRGB24Pixel* pImage = pImageRow;
		XnRGB24Pixel* pTex = pTexRow + g_imageMD.XOffset();
		for (XnUInt x = 0; x < g_imageMD.XRes(); ++x, ++pImage, ++pTex){
			*pTex = *pImage;
		}
		pImageRow += g_imageMD.XRes();
		pTexRow += g_nTexMapX;
	}

	const XnDepthPixel* pDepthRow = g_depthMD.Data();
	XnRGB24Pixel* pTexRow_depth = g_pTexMap + g_depthMD.YOffset() * g_nTexMapX;

	XnPoint3D* tot_pt = new XnPoint3D[nNumberOfPoints];
	XnPoint3D* tot_mt = new XnPoint3D[nNumberOfPoints];
	XnRGB24Pixel* tot_col = new XnRGB24Pixel[nNumberOfPoints];
	int i=0;

	//DEPTH
	for (XnUInt y = 0; y < g_depthMD.YRes(); ++y){
		const XnDepthPixel* pDepth = pDepthRow;
		XnRGB24Pixel* pTex = pTexRow_depth + g_depthMD.XOffset();

		for (XnUInt x = 0; x < g_depthMD.XRes(); ++x, ++pDepth, ++pTex){
			if (*pDepth!=0 ){
				XnPoint3D p;
				p.X = x; p.Y = y; p.Z = *pDepth;
				tot_pt[i] = p;
				tot_col[i] = *pTex;
				++i;
				}
		}
				pDepthRow += g_depthMD.XRes();
				pTexRow_depth += g_nTexMapX;
	}


	xnConvertProjectiveToRealWorld(g_depth,nNumberOfPoints,tot_pt,tot_mt);

	for(int k=0; k<nNumberOfPoints; k++){
		fprintf(ply,"%f %f %f %u %u %u \n",(float)tot_mt[k].X,(float)tot_mt[k].Y,-(float)tot_mt[k].Z, tot_col[k].nRed,tot_col[k].nGreen,tot_col[k].nBlue);
	}

	fclose(ply);
	//reset the sensor viewpoint
	g_depth.GetAlternativeViewPointCap().ResetViewPoint();
	

}

//filename must end in ply
void Kinect::savePointCloud(RGBDpixel** rgbd,char* filename){

	char* ofilename = filename;

	char str[100];
	int s=0;
	while(*filename != NULL){

		str[s] = *filename++;
		s++;

	}

    char *test[10];
    test[0] = strtok(str, "."); // Splits spaces between words in str
    test[1] = strtok (NULL, ".");

	if(strcmp(test[1],"ply")!= 0){

		fprintf(stderr,"Specified wrong format..appending .ply to the filename\n");
		char new_filename[80];
		strcpy(new_filename,test[0]);
		strcat(new_filename,".ply");
		ofilename = new_filename;
		fprintf(stderr,"New filename is %s\n",ofilename);
	}
	


	unsigned int nNumberOfPoints = 640*480;

	FILE* ply = fopen(ofilename,"w");
	/*write header*/
	fprintf(ply,"ply\n");
	fprintf(ply,"format ascii 1.0\n");
	fprintf(ply,"element vertex %d\n",nNumberOfPoints);
	fprintf(ply,"property float x\n");
	fprintf(ply,"property float y\n");
	fprintf(ply,"property float z\n");
	fprintf(ply,"property uchar diffuse_red\n");
	fprintf(ply,"property uchar diffuse_green\n");
	fprintf(ply,"property uchar diffuse_blue\n");
	fprintf(ply," end_header\n");

	for(int i=0; i<640; i++){
		for(int j=0; j<480; j++){
			fprintf(ply,"%f %f %f %u %u %u \n",rgbd[i][j].x,rgbd[i][j].y,-rgbd[i][j].z, (unsigned char) rgbd[i][j].r, (unsigned char) rgbd[i][j].g, (unsigned char) rgbd[i][j].b);
		}
	}

	fclose(ply);
	

}

RGBDpixel** Kinect::extractRGBDpoints(unsigned char* pImage, unsigned short* pDepth, int& tot_non_zero){

	RGBDpixel** pixelsOut = new RGBDpixel*[width];
	for (int i = 0; i < width; i++) {
		pixelsOut[i] = new RGBDpixel[height];
	}

	tot_non_zero = 0;



	unsigned int nNumberOfPoints = width*height;

	//RGB
	const XnRGB24Pixel* pImageRow = g_imageMD.RGB24Data();
	XnRGB24Pixel* pTexRow = g_pTexMap + g_imageMD.YOffset() * g_nTexMapX;

	int g= g_imageMD.YRes();

	for (XnUInt y = 0; y < g_imageMD.YRes(); ++y){
		const XnRGB24Pixel* pImage = pImageRow;
		XnRGB24Pixel* pTex = pTexRow + g_imageMD.XOffset();
		for (XnUInt x = 0; x < g_imageMD.XRes(); ++x, ++pImage, ++pTex){
			*pTex = *pImage;
		}
		pImageRow += g_imageMD.XRes();
		pTexRow += g_nTexMapX;
	}

	const XnDepthPixel* pDepthRow = g_depthMD.Data();
	XnRGB24Pixel* pTexRow_depth = g_pTexMap + g_depthMD.YOffset() * g_nTexMapX;

	XnRGB24Pixel* tot_col = new XnRGB24Pixel[nNumberOfPoints];
	int i=0;

	//DEPTH
	float wx, wy, wz;
	float z;
	for (XnUInt y = 0; y < g_depthMD.YRes(); ++y){
		const XnDepthPixel* pDepth = pDepthRow;
		XnRGB24Pixel* pTex = pTexRow_depth + g_depthMD.XOffset();
		for (XnUInt x = 0; x < g_depthMD.XRes(); ++x, ++pTex){
 
			tot_col[i] = *pTex;
			//RGBDpixel rgbdp;
			pixelsOut[x][y].r = tot_col[i].nRed;
			pixelsOut[x][y].g = tot_col[i].nGreen;
			pixelsOut[x][y].b = tot_col[i].nBlue;
			
			z = (float)pDepth[x];
			if (z > 0.1f) {
				//convertToWorld(x,y,z,wx,wy,wz);
				wx = (x/640.0f-0.5f)*z*1.1114666461944f;
				wy = (0.5f-y/480.0f)*z*0.8336956551416f;
				wz = z;
				tot_non_zero++;
			} else {
				wx = (float) y;
				wy = (float) z;
				wz = (float) z;
			}

			//wx = (x/640.0f-0.5f)*z*0.0011114666461944f;
			//wy = (0.5f-y/480.0f)*z*0.0008336956551416f;
			//wz = z/1000.0f;

			pixelsOut[x][y].x = wx;
			pixelsOut[x][y].y = wy;
			pixelsOut[x][y].z = wz;
			pixelsOut[x][y].d = z;
			++i;
		}
		pDepthRow += g_depthMD.XRes();
		pTexRow_depth += g_nTexMapX;
	}

	delete[] tot_col;
	tot_col = NULL;
	return pixelsOut;

}

RGBDpixel** Kinect::extractRGBDpointsIntoPCL(unsigned char* pImage, unsigned short* pDepth, int& tot_non_zero, pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud){

	RGBDpixel** pixelsOut = new RGBDpixel*[width];
	for (int i = 0; i < width; i++) {
		pixelsOut[i] = new RGBDpixel[height];
	}

	tot_non_zero = 0;

	unsigned int nNumberOfPoints = width*height;

	//RGB
	const XnRGB24Pixel* pImageRow = g_imageMD.RGB24Data();
	XnRGB24Pixel* pTexRow = g_pTexMap + g_imageMD.YOffset() * g_nTexMapX;

	int g= g_imageMD.YRes();

	for (XnUInt y = 0; y < g_imageMD.YRes(); ++y){
		const XnRGB24Pixel* pImage = pImageRow;
		XnRGB24Pixel* pTex = pTexRow + g_imageMD.XOffset();
		for (XnUInt x = 0; x < g_imageMD.XRes(); ++x, ++pImage, ++pTex){
			*pTex = *pImage;
		}
		pImageRow += g_imageMD.XRes();
		pTexRow += g_nTexMapX;
	}

	const XnDepthPixel* pDepthRow = g_depthMD.Data();
	XnRGB24Pixel* pTexRow_depth = g_pTexMap + g_depthMD.YOffset() * g_nTexMapX;

	XnRGB24Pixel* tot_col = new XnRGB24Pixel[nNumberOfPoints];
	int i=0;

	//DEPTH
	float wx, wy, wz;
	float z;

	pcl_cloud.width = 640;
	pcl_cloud.height = 480;
	pcl_cloud.is_dense = false;
	pcl_cloud.points.resize(nNumberOfPoints); //assuming you used ALL the points returned from the kinect when building up RGBDpixel**

	for (XnUInt y = 0; y < g_depthMD.YRes(); ++y){
		const XnDepthPixel* pDepth = pDepthRow;
		XnRGB24Pixel* pTex = pTexRow_depth + g_depthMD.XOffset();
		for (XnUInt x = 0; x < g_depthMD.XRes(); ++x, ++pTex){
 
			tot_col[i] = *pTex;
			//RGBDpixel rgbdp;
			pixelsOut[x][y].r = tot_col[i].nRed;
			pixelsOut[x][y].g = tot_col[i].nGreen;
			pixelsOut[x][y].b = tot_col[i].nBlue;
			
			z = (float)pDepth[x];
			//if (z > 0.1f) {
				//convertToWorld(x,y,z,wx,wy,wz);
				wx = (x/640.0f-0.5f)*z*1.1114666461944f;
				wy = (0.5f-y/480.0f)*z*0.8336956551416f;
				wz = z;
				//do coords
				pcl_cloud.points[tot_non_zero].x = wx/1000.0f;
				pcl_cloud.points[tot_non_zero].y = wy/1000.0f;
				pcl_cloud.points[tot_non_zero].z = wz/1000.0f;
				//do rgb
			    uint8_t r = (uint8_t)pixelsOut[x][y].r; 
			    uint8_t g = (uint8_t)pixelsOut[x][y].g; 
			    uint8_t b = (uint8_t)pixelsOut[x][y].b;   	
			    int32_t rgb_val = (r << 16) | (g << 8) | b;
			    pcl_cloud.points[tot_non_zero].rgb = *(float *)(&rgb_val); 
				tot_non_zero++;
				/*
			} else {
				wx = (float) y;
				wy = (float) z;
				wz = (float) z;
			}
      */
      
			pixelsOut[x][y].x = wx;
			pixelsOut[x][y].y = wy;
			pixelsOut[x][y].z = wz;
			pixelsOut[x][y].d = z;
			++i;
		}
		pDepthRow += g_depthMD.XRes();
		pTexRow_depth += g_nTexMapX;
	}

	pcl_cloud.points.resize(tot_non_zero);

	delete[] tot_col;
	
	for (int i = 0; i < width; i++) {
		delete[] pixelsOut[i];
	}
	delete[] pixelsOut;
	
	tot_col = NULL;
	return pixelsOut;

}


	/*RGBDpixel** pixelsOut = new RGBDpixel*[width];
	for (int i = 0; i < width; i++) {
		pixelsOut[i] = new RGBDpixel[height];
	}
	unsigned int nNumberOfPoints = width*height;

	//RGB
	const XnRGB24Pixel* pImageRow = g_imageMD.RGB24Data();
	XnRGB24Pixel* pTexRow = g_pTexMap + g_imageMD.YOffset() * g_nTexMapX;

	int g= g_imageMD.YRes();

	for (XnUInt y = 0; y < g_imageMD.YRes(); ++y){
		const XnRGB24Pixel* pImage = pImageRow;
		XnRGB24Pixel* pTex = pTexRow + g_imageMD.XOffset();
		for (XnUInt x = 0; x < g_imageMD.XRes(); ++x, ++pImage, ++pTex){
			*pTex = *pImage;
		}
		pImageRow += g_imageMD.XRes();
		pTexRow += g_nTexMapX;
	}

	const XnDepthPixel* pDepthRow = g_depthMD.Data();
	XnRGB24Pixel* pTexRow_depth = g_pTexMap + g_depthMD.YOffset() * g_nTexMapX;

	XnPoint3D* tot_pt = new XnPoint3D[nNumberOfPoints];
	XnPoint3D* tot_mt = new XnPoint3D[nNumberOfPoints];
	XnPoint3D* otot_mt = tot_mt;
	XnRGB24Pixel* tot_col = new XnRGB24Pixel[nNumberOfPoints];
	int i=0;

	//DEPTH
	for (XnUInt y = 0; y < g_depthMD.YRes(); ++y){
		const XnDepthPixel* pDepth = pDepthRow;
		XnRGB24Pixel* pTex = pTexRow_depth + g_depthMD.XOffset();
		for (XnUInt x = 0; x < g_depthMD.XRes(); ++x, ++pTex){
 
			XnPoint3D p;
			p.X = x; p.Y = y; p.Z = pDepth[x];
			tot_pt[i] = p;
			tot_col[i] = *pTex;
			//RGBDpixel rgbdp;
			pixelsOut[x][y].r = tot_col[i].nRed;
			pixelsOut[x][y].g = tot_col[i].nGreen;
			pixelsOut[x][y].b = tot_col[i].nBlue;
			//pixelsOut[y][x].x = p.X;
			//pixelsOut[y][x].y = p.Y;
			//pixelsOut[y][x].z = p.Z;
			pixelsOut[x][y].d = p.Z;
			//pixelsOut[y][x] = rgbdp;
			++i;
		}
				pDepthRow += g_depthMD.XRes();
				pTexRow_depth += g_nTexMapX;
	}


	xnConvertProjectiveToRealWorld(g_depth,nNumberOfPoints,tot_pt,tot_mt);
	XnPoint3D temp_p;
	for (XnUInt y = 0; y < g_depthMD.YRes(); ++y){
		for (XnUInt x = 0; x < g_depthMD.XRes(); ++x){
			temp_p = *tot_mt++;
			pixelsOut[x][y].x = temp_p.X;
			pixelsOut[x][y].y = temp_p.Y;
			pixelsOut[x][y].z = temp_p.Z;
		}
	}

	delete[] tot_pt;
	delete[] otot_mt;
	delete[] tot_col;
	
	return pixelsOut;
	*/

void inline Kinect::convertToWorld(float kx, float ky, float kz, float& wx, float& wy, float& wz) {
	//got from OpenNI: http://groups.google.com/group/openni-dev/browse_thread/thread/c2b37ca04ff55257/dc7172d100f122ae?lnk=gst&q=ConvertRealWorldToProjective#dc7172d100f122ae
    //float FovH=1.0144686707507438; 
	//float FovV=0.78980943449644714;

	//float Xres = 640.0f;
	//float Yres = 480.0f;

    // got from OpenNI: http://groups.google.com/group/openni-dev/browse_thread/thread/c2b37ca04ff55257/dc7172d100f122ae?lnk=gst&q=ConvertRealWorldToProjective#dc7172d100f122ae
	//float XtoZ=tan(FovH/2.0f)*2.0f;
	//float YtoZ=tan(FoVV/2.0f)*2.0f;

	//got from OpenNI: http://groups.google.com/group/openni-dev/browse_thread/thread/c2b37ca04ff55257/dc7172d100f122ae?lnk=gst&q=ConvertRealWorldToProjective#dc7172d100f122ae
	//wx = (kx/640.0f-0.5f)*kz*XtoZ/1000.0f;
	//wy = (0.5f-ky/480.0f)*kz*YtoZ/1000.0f; 
	//wz= kz/1000.0f;

	wx = (kx/640.0f-0.5f)*kz*0.0011114666461944;
	wy = (0.5f-ky/480.0f)*kz*0.0008336956551416;
	wz = kz/1000.0f;
}

void Kinect::deleteRGBDpoints(RGBDpixel** pixels) {
	for (int i = 0; i < width; i++) {
		delete[] pixels[i];
		pixels[i] = NULL;
	}
	delete[] pixels;
	pixels = NULL;
}

RGBDpixel Kinect::RGBDpointAt(int i, int j, std::vector<RGBDpixel> buf){

	RGBDpixel pixel;
	int index = (((j-1)*640)+i)-1;
	pixel = buf[index];
	return pixel;

}


XnPoint3D* Kinect::projective2meters(unsigned short* pDepth){

	int nNumberOfPoints = width*height;

	XnPoint3D* tot_pt = new XnPoint3D[nNumberOfPoints];
	XnPoint3D* tot_mt = new XnPoint3D[nNumberOfPoints];

	int i = 0;

	//DEPTH
	for (XnUInt x = 0; x < g_depthMD.XRes(); ++x){
		for (XnUInt y = 0; y < g_depthMD.YRes(); ++y, ++pDepth, ++i){
				XnPoint3D p;
				p.X = (float)x; p.Y = (float)y; p.Z = ((float)*pDepth);
				tot_pt[i] = p;
		}
	}


	xnConvertProjectiveToRealWorld(g_depth,nNumberOfPoints,tot_pt,tot_mt);

	return tot_mt;



	/* If you need to use these points together with the colour image rgb (i.e to write out a point cloud) you can do the following 

	int j = 0;
	for(int x=0: x<width*height; x++){
		unsigned char r = *rgb++;
		unsigned char g = *rgb++;
		unsigned char b = *rgb++
		XnPoint3D = tot_mt[j];

		//now you have r,g,b together with X,Y,Z in meters!! Do the stuff you wanna do

		j++;
	}

	*/

}

float* Kinect::projective2metersImage(unsigned short* pDepth){

	int nNumberOfPoints = width*height;
	float* img = new float[nNumberOfPoints];
	XnPoint3D* points = projective2meters(pDepth);
	for(int i=0; i < nNumberOfPoints; i++){
			img[i] = points[i].Z;
	}

	return img;

}

bool Kinect::saveImage(char* filename){

	/* to implement*/
	fprintf(stderr,"To implement!\n");
	return true;

}
