/**************************************************************
Intial demo of rendering to sphere using an offscreen buffer.

Need fixes
- top and bottom aren't working - done
- need to integrate avatar - done
- fix lighting demonstration - done

Also, this is not the right way to get an accurate head-tracking 
demo on the sphere. It can't use standard environment mapping.
However, it will be approximately correct.

***************************************************************/
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#if defined(_WIN32)
#include <windows.h>
#else
#include<unistd.h>
#include<cstdlib>
#endif
//SYS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <glee.h>
#include <vector>
#include <map>
#include <iostream>
//OpenGL
#include <GL/glut.h>
//Networking
#include "client.h"
//3D Maths
#include <math.h>
#include "utilMath.h"
//Kinect
#include "Kinect.h"
//QT
#define QSTREAM	//comment to disable link to client manager
#ifdef QSTREAM //client-manager manages app
	//#include <QApplication>
	#include <QBuffer>
	#include <QDataStream>
	#include <QSharedMemory>
	#include <QTextStream>
	#include <QStringList>
	#ifdef _DEBUG
		// debug version…
		#pragma comment ( lib, "QtCored4.lib")
		//#pragma comment ( lib, "QtGuid4.lib")
	#else
		// release version.
		#pragma comment ( lib, "QtCore4.lib")
		//#pragma comment ( lib, "QtGui4.lib")
	#endif
	#define OFFLINE_TEST 1
	QSharedMemory clientMemory;	//establish shared memory
	QString avatarCFG;
#else //stand-alone app
	#define OFFLINE_TEST 0
#endif

#define COLOUR_BACKGROUND 0

#define DEGTORAD (3.141596/180)
#define RADTODEG (180/3.141596)

GLuint color_tex;
GLuint fb;
GLuint depth_rb;

#include "trackball.h"  /* trackball quaternion routines */

/* Menu items. */
enum {
	M_TEST, M_INSIDE, M_OUTSIDE
};

std::map<std::string, std::vector<std::string> > new_clients;
std::map<std::string, int > avatars;
int cameramode = M_INSIDE;
bool front_view = false;

int spinning = 0, moving = 0;
int beginx, beginy;
int W = 640, H = 480;
float curquat[4];
float lastquat[4];
bool y_rot = false, replay1 = false, replay2 = false, auto_frustum = true;
float outsideCamY=-90.0f;
float rot_angle = 0.0, rotx=0.0, roty=0.0, rotz=1.0;
char visitor[128];
char local[128];
char halca_dir[128];
char which_avatar[128];
int visitor_headnode=0;	
int local_headnode=5;		
//float translate_x=0.f, translate_y=-0.8f, translate_z=-5.0f;
float translate_x=0.f, translate_y=-0.8f, translate_z=0.0f;
float scale_outside=3.6f;
float scale_inside=54.5f;
int   rotating=0;
float inside_headY=0.f, outside_headY=0.f, front_headY=0.f;
float inside_sphereY=0.f, outside_sphereY=0.f, front_sphereY=0.f;
int reverseX=1, reverseY=1, reverseZ=1, constrain_head_rot=0;
float constrainAngle = 0.f, constrain_head_rx=0.f, constrain_head_ry=1.f, constrain_head_rz=0.f, constrain_head_rw=0.f;
float eye_rx=0.f, eye_ry=1.f, eye_rz=0.f, eye_rw=0.f;
float head_x=0.f, head_y=0.f, head_z=0.f, head_rx=0.f, head_ry=1.f, head_rz=0.f, head_rw=0.f;
float local_x=0.f, local_y=0.f, local_z=0.f, local_rx=0.f, local_ry=1.f, local_rz=0.f, local_rw=0.f;
float jaw_x=0.f;
float offset_x=0.f, offset_y=0.f, offset_z=0.f;

typedef int (__cdecl*HALCA_INITCAL3D)(char*);
typedef int (__cdecl*HALCA_ADDCHARACTER)(char*, char*);
typedef void (__cdecl*HALCA_SETTRANSFORMTYPE)(int);
typedef void (__cdecl*HALCA_DRAW)(float);
typedef void (__cdecl*HALCA_IDLE)();
typedef void (__cdecl*HALCA_EXEACT)(int ,int , float , float , float , int );
typedef int  (__cdecl*HALCA_INITHALCA)(char*);
typedef void (__cdecl*HALCA_SETROTATION)(int,int,float*);
typedef void (__cdecl*HALCA_ADDROTATION)(int,int,float*);
typedef void (__cdecl*HALCA_SETROTATIONEULER)(int,int,float*);
typedef void (__cdecl*HALCA_ADDROTATIONEULER)(int,int,float*);
typedef void (__cdecl*HALCA_SETTRANSLATION)(int,int,float*);
typedef void (__cdecl*HALCA_GETTRANSLATIONABS)(int,int,float*);
typedef void (__cdecl*HALCA_GETTRANSLATION)(int,int,float*);
typedef void (__cdecl*HALCA_GETTRANSLATIONOGL)(int,int,float*);
typedef void (__cdecl*HALCA_RESETBONE)(int,int);
typedef int (__cdecl*HALCA_GETNUMBONE)(int);

HALCA_INITCAL3D halca_initCal3D;
HALCA_ADDCHARACTER halca_addCharacter;
HALCA_SETTRANSFORMTYPE halca_setTransformType;
HALCA_DRAW halca_draw;
HALCA_IDLE halca_idle;
HALCA_EXEACT halca_exeAct;
HALCA_INITHALCA halca_initHALCA;
HALCA_SETROTATION halca_setRotation;
HALCA_ADDROTATION halca_addRotation;
HALCA_SETROTATIONEULER halca_setRotationEuler;
HALCA_ADDROTATIONEULER halca_addRotationEuler;
HALCA_SETTRANSLATION halca_setTranslation;
HALCA_GETTRANSLATIONABS halca_getTranslationAbs;
HALCA_GETTRANSLATION halca_getTranslation;
HALCA_GETTRANSLATIONOGL halca_getTranslationOGL;
HALCA_RESETBONE halca_resetBone;
HALCA_GETNUMBONE halca_getNumBone;
int id;
float lipsync[3] = {0.f,0.f,0.f};
float allbones_pos[369] = {0};

//OPENGL
#define MAXVIEWSTEPS 5
/*int window; 
int w_width = 800;
int w_height = 600;
bool draw_plane = false;
int frame_w = 640;
int frame_h = 480;*/
int channel_size = 640*480;
unsigned char* rgb = new unsigned char[channel_size*3];
unsigned short* depth = new unsigned short[channel_size];
bool viewingPoints = true;
int views = 0;
//this brings things in front of the camera
float zoom = 0.0f;
GLfloat rotateX = 0.0f;
GLfloat rotateY = 0.0f;
GLfloat centerZ = 0.0f;
int maxViews = MAXVIEWSTEPS;
std::vector<GLuint> list(maxViews);
//PCL
pcl::PointCloud<pcl::PointXYZRGB>gpc;
Kinect* kin;
float max_dist = 1.5f;
bool point_cloud_enabled = false;
bool kin_running;

///////////////////////////////////////////////////////////////////////////////
// initialise HALCA
///////////////////////////////////////////////////////////////////////////////
void initHALCA(){
	HINSTANCE hLib=LoadLibraryW(L"HALCAWin32.dll");
	halca_initHALCA=(HALCA_INITHALCA)GetProcAddress(hLib,"initHALCA");
	//halca_initCal3D=(HALCA_INITCAL3D)GetProcAddress(hLib,"initCal3D");
	halca_addCharacter=(HALCA_ADDCHARACTER)GetProcAddress(hLib,"addCharacter");
	halca_setTransformType=(HALCA_SETTRANSFORMTYPE)GetProcAddress(hLib,"setTransformType");
	halca_draw=(HALCA_DRAW)GetProcAddress(hLib,"Draw");
	halca_exeAct=(HALCA_EXEACT)GetProcAddress(hLib,"exeAct");
	halca_setTranslation=(HALCA_SETTRANSLATION)GetProcAddress(hLib,"setTranslation");
	halca_getTranslationAbs=(HALCA_GETTRANSLATIONABS)GetProcAddress(hLib,"getTranslationAbs");
	halca_getTranslation=(HALCA_GETTRANSLATION)GetProcAddress(hLib,"getTranslation");
	halca_getTranslationOGL=(HALCA_GETTRANSLATIONOGL)GetProcAddress(hLib,"getTranslationOGL");
	halca_setRotation=(HALCA_SETROTATION)GetProcAddress(hLib,"setRotation");
	halca_addRotation=(HALCA_ADDROTATION)GetProcAddress(hLib,"addRotation");
	halca_resetBone=(HALCA_RESETBONE)GetProcAddress(hLib,"resetBone");
	halca_getNumBone=(HALCA_GETNUMBONE)GetProcAddress(hLib,"getNumBone");
	halca_idle=(HALCA_IDLE)GetProcAddress(hLib,"Idle");

	// !!!!!!!!!! set path to parent of directory containing avatar data !!!!!!!!!!!!
#ifdef QSTREAM
	halca_initHALCA(halca_dir);
	halca_setTransformType(0);
	char av[128]; strcpy(av,which_avatar); strcat(av,".cfg");
	id = halca_addCharacter(which_avatar,av);
#else
	//halca_initHALCA("C:/Users/anthony/Desktop/beaming-svn/netManager/sampleXVRclient/");
	//halca_initHALCA("C:/Beaming/trunk/netManager/sampleXVRclient/");
	//halca_initHALCA("C:/wole/Beaming/netManager/sampleXVRclient/");
	halca_initHALCA("E:/wole/Beaming/avatarViewers/xvr_test/");
	halca_setTransformType(0);
	id = halca_addCharacter("m016_head","m016_head.cfg");
#endif

	halca_idle();

    halca_getTranslation(id,13,lipsync); //moves the jaw - lip sync
	float tmp_bone[3] = {0};
	for ( int i=0; i< halca_getNumBone(id); i++)
	{
		//printf("%d,%d\t",i,i*3+1);
		halca_getTranslation(id,i,tmp_bone);
		allbones_pos[i*3] = tmp_bone[0];
		allbones_pos[i*3+1] = tmp_bone[1];
		allbones_pos[i*3+2] = tmp_bone[2];
		//char tmp[128];
		//sprintf(tmp,"%d - %.4f,%.4f,%.4f",i,allbones_pos[i*3],allbones_pos[i*3+1],allbones_pos[i*3+2]);
		//std::cout << tmp << std::endl;
	}
}


/*
	Fills a glList list with pc data
*/
void drawInList(pcl::PointCloud<pcl::PointXYZRGB>& pc) 
{

	  list[views] = glGenLists(1);
	  glNewList(list[views], GL_COMPILE);

	  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	  glPointSize(1.0f);

	  glBegin(GL_POINTS);
	  for (int j = 0; j < pc.points.size(); j++) 
	  {
		  if(fabs(pc.points[j].z-centerZ)<max_dist)
		  {
		    int32_t rgb_val = *(int32_t*)(&pc.points[j].rgb);

		    float r = (float) ((unsigned char) (rgb_val >> 16)) / 255.0f;
		    float g = (float) ((unsigned char) (rgb_val >> 8)) / 255.0f;
		    float b = (float) ((unsigned char) (rgb_val >> 0)) / 255.0f;
		
		    glColor3f(r,g,b);
		    glVertex3f(pc.points[j].x,pc.points[j].y,-(pc.points[j].z-centerZ));
		  }

	  }
	  glEnd();
	  glEndList();
	  views++;
}


void drawCloudInlist(pcl::PointCloud<pcl::PointXYZRGB>& pc, bool global)
{

    	if (views == 0) 
	{
		float z = 0.0f;
		int zcount = 0;
	    	for (int j = 0; j < pc.points.size(); j=j+100) 
		{
	      		z+= pc.points[j].z;
			zcount++;
		}
	
		centerZ = z / (float) zcount;
  	}

  	if (global) 
	{
		for (int i = 0; i < list.size(); i++) 
		{
	      		glDeleteLists(list[i],1);
		}
	    	list.resize(MAXVIEWSTEPS);
			views = 0;	
		} 
		else 
		{
			if (views == maxViews) 
			{
				maxViews += MAXVIEWSTEPS;
	      			list.resize(maxViews);
			}
	  }

	  drawInList(pc);
	  viewingPoints = true;
	  glutPostRedisplay();

  
}

void
setRenderCubemapToSphereState(void)
{
	// This is the state required to render the cubemap to the sphere.
	glTexParameteri(GL_TEXTURE_CUBE_MAP_EXT, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP_EXT, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glEnable(GL_TEXTURE_CUBE_MAP_EXT);
	glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_REFLECTION_MAP_EXT);
	glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_REFLECTION_MAP_EXT);
	glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_REFLECTION_MAP_EXT);
	glTexParameteri(GL_TEXTURE_CUBE_MAP_EXT, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP_EXT, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glEnable(GL_TEXTURE_GEN_S);
	glEnable(GL_TEXTURE_GEN_T);
	glEnable(GL_TEXTURE_GEN_R);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDisable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glDisable(GL_CULL_FACE);
}

void
setRenderSceneToCubemapState(void)
{
	// This is the state required to render the base scene that we want to show on the sphere.
	// E.G. to draw an avatar head.
	glDisable(GL_TEXTURE_CUBE_MAP_EXT);
	glDisable(GL_TEXTURE_GEN_S);
	glDisable(GL_TEXTURE_GEN_T);
	glDisable(GL_TEXTURE_GEN_R);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glCullFace(GL_NONE);
	switch(cameramode)
	{
	case M_INSIDE:
		glDepthFunc(GL_GEQUAL);
		break;
	case M_TEST:
	case M_OUTSIDE:
		if (point_cloud_enabled)
			glDepthFunc(GL_LESS);
		else
			glDepthFunc(GL_LEQUAL);
		break;
	}
}

void
renderBoxes(void)
{
	glColor3f(0,0,0);
	glutWireCube(0.5);
	glColor3f(1,1,1);
	glutWireCube(1.0);
}

void
renderHead(void)
{
	//halca display start
    float arr[3] = {translate_x,translate_y,translate_z};//{0.f,-0.8f,4.0f};
    halca_setTranslation(id,0,arr); //Translates the avatar's body accordingly
    arr[0]=0.f; arr[1]=1.f; arr[2]=0.f;
    float q[4];
    axis_to_quat(arr,-90*DEGTORAD,q);
    halca_setRotation(id,0,q); //rotates the avatar's body accordingly

	float lip[3];
#ifdef QSTREAM
	if (replay1)
	{
		lip[0]=jaw_x;
		lip[1]=0.f;
		lip[2]=0.f;
	} else {
		lip[0]=lipsync[0]+jaw_x;
		lip[1]=lipsync[1]+0.f;
		lip[2]=lipsync[2]+0.f;
	}
#else
	lip[0]=lipsync[0]+jaw_x;
	lip[1]=lipsync[1]+0.f;
	lip[2]=lipsync[2]+0.f;
#endif
    //halca_setTranslation(id,13,lip); //moves the jaw - lip sync
	for ( int i=13; i<=63; i++)//update local positions of bones 13 to 63 (face nodes).
	{
		float arr[3] = {allbones_pos[i*3],allbones_pos[i*3+1],allbones_pos[i*3+2]};
		halca_setTranslation(id,i,arr); 
	}

	glPushAttrib(GL_ALL_ATTRIB_BITS);

	/*glBegin(GL_LINES);
	glColor3f(1,0,0);
	glVertex3f(0,0,0);
	glVertex3f(1,0,0);

	glColor3f(0,1,0);
	glVertex3f(0,0,0);
	glVertex3f(0,1,0);

	glColor3f(0,0,1);
	glVertex3f(0,0,0);
	glVertex3f(0,0,1);
	glEnd();*/
	glFlush();
	glPushMatrix();
	switch(cameramode)
	{
	case M_INSIDE:
		glScalef(scale_inside,scale_inside,scale_inside);
		break;
	case M_TEST:
	case M_OUTSIDE:
		glScalef(scale_outside,scale_outside,scale_outside);
		break;
	}
	glPushMatrix();
	if ((cameramode==M_OUTSIDE)&&(!front_view))	{
		glRotatef(outside_headY, 0, 1, 0);
	}
	else if ((cameramode==M_OUTSIDE)&&(front_view))	{
		glRotatef(front_headY, 0, 1, 0);
	}
	halca_draw(0.0);
	glPopMatrix();
	glPopMatrix();
	glPopAttrib();
	//halca display end
}

/*
	Called on rendering
*/
void renderPointCloud(void)
{
	//get a kinect frame
	int non_zero_pts = 0;
	rgb = (unsigned char*)kin->grabRGB();
	depth = (unsigned short *)kin->grabDepth();
	RGBDpixel** rgbd_buff = kin->extractRGBDpointsIntoPCL(rgb, depth, non_zero_pts,gpc);
	drawCloudInlist(gpc, true);	

  	//glutMakeCurrent();
	//glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearDepth(1.0);				// Enables Clearing Of The Depth Buffer
	glDepthFunc(GL_LESS);			        // The Type Of Depth Test To Do
	glEnable(GL_DEPTH_TEST);		        // Enables Depth Testing
	glDisable(GL_LIGHTING);
  	glDisable(GL_BLEND);
	glShadeModel(GL_SMOOTH);			// Enables Smooth Color Shading
	//glViewport(0,0,640,480);
	//glMatrixMode(GL_PROJECTION);
	//glLoadIdentity();
	//gluPerspective(60.0f, 1.0f, 0.2f, 100.0f);

	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();

	//glTranslatef(0.0f,0.0f,zoom);
	//this is controlled with the mouse
	//glRotatef(rotateX,0.0f,1.0f,0.0f);
	//glRotatef(-rotateY,1.0f,0.0f,0.0f);
	if (views != 0) 
	{
		//draw cloud
		for (int i = 0; i < views; i++)
    		glCallList(list[i]);
	}
	glFlush();

}

void
setRenderCubemapToSphereMatrices()
{

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective( /* field of view in degree */ 22.5,
		/* aspect ratio */ 1.4,
		/* Z near */ 1.0, /* Z far */ 10.0);
	glViewport(0,0,W,H);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0, 0.0, 5.0,  /* eye is at (0,0,5) */
		0.0, 0.0, 0.0,      /* center is at (0,0,0) */
		0.0, 1.0, 0.);      /* up is in positive Y direction */

	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();
	glScalef(-1,-1,-1);
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	if ((cameramode==M_OUTSIDE)&&(front_view))	{
		glRotatef(front_sphereY, 0.0, 0.0, 1.0);
		glRotatef(rot_angle, rotx, roty, rotz);	
	}
	else if ((cameramode==M_OUTSIDE)&&(!front_view))	{
		glRotatef(outside_sphereY, 0, 0, 1);
	}
	else if (cameramode==M_INSIDE) {
		glRotatef(inside_sphereY, 0.0, 0.0, 1.0);
	}
	glMatrixMode(GL_MODELVIEW);
}

#define ZNEAR 0.1f
#define ZFAR  10.0f

void
setRenderSceneToCubemapMatrices(GLuint screen=GL_TEXTURE_CUBE_MAP_POSITIVE_X)
{
	float camx, camy, camz;

	switch(cameramode)
	{
	case M_TEST:
		// In this mode all the views are the same, just to prove that there is an object rendering!
		// You can manipulate the object with the mouse
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective( /* field of view in degree */ 22.5,
			/* aspect ratio */ 1.4f,
			/* Z near */ 0.1, /* Z far */ 100.0);
		glViewport(0,0,256,256);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(0.0, 1.0, 1.0,  /* eye is at (0,0,5) */
			0.0, 1.0, 0.0,      /* center is at (0,0,0) */
			0.0, 1.0, 0.0);      /* up is in positive Y direction */
		break;
	case M_OUTSIDE:

		camx = cosf(DEGTORAD*outsideCamY)*3.0f;
		camy = 0.0f;
		camz = sinf(DEGTORAD*outsideCamY)*3.0f;
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		float cx, cy, cz;
		switch(screen)
		{
			// Easiest way to work this out is to calculate the cam coordinate in a screen centred coordinate system
			/// then the glFrustum call is the same (see below).
		case GL_TEXTURE_CUBE_MAP_POSITIVE_X:
			cx = -camz; cy = camy; cz = camx;
			break;
		case GL_TEXTURE_CUBE_MAP_NEGATIVE_X:
			cx = camz; cy = camy; cz = -camx;
			break;
		case GL_TEXTURE_CUBE_MAP_POSITIVE_Y:
			cx = -camx; cy = camz; cz = -camy;
			break;
		case GL_TEXTURE_CUBE_MAP_NEGATIVE_Y:
			cx = -camx; cy = -camz; cz = camy;
			break;

		case GL_TEXTURE_CUBE_MAP_POSITIVE_Z:
			cx = camx; cy = camy; cz = camz;
			break;
		case GL_TEXTURE_CUBE_MAP_NEGATIVE_Z:
			cx = -camx; cy = camy; cz = -camz;
			break;
		}
		if (cz-0.5 > 0)
		{
			glFrustum(-0.5-cx, 0.5-cx, -0.5-cy, 0.5-cy, cz-0.5, ZFAR);
		}
		else
		{
			glFrustum(-0.5-cx, 0.5-cx, -0.5-cy, 0.5-cy, ZFAR, ZFAR);
		}
#if 1
		/* Recover the current matrix */
		GLfloat projectionMatrix[16];
		glGetFloatv(GL_PROJECTION_MATRIX,projectionMatrix);
		/* Modify the entries */
		projectionMatrix[10]=(-ZFAR-ZNEAR)/(ZFAR-ZNEAR);
		projectionMatrix[14]=(-2*ZFAR*ZNEAR)/(ZFAR-ZNEAR);
		/* Load the modified matrix */
		glLoadMatrixf(projectionMatrix);
#endif
		glViewport(0,0,256,256);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		switch(screen)
		{
		case GL_TEXTURE_CUBE_MAP_NEGATIVE_X:
			gluLookAt(camx, camy, camz, 0.0, camy, camz, 0.0, 1.0, 0.);
			break;
		case GL_TEXTURE_CUBE_MAP_POSITIVE_X:
			gluLookAt(camx, camy, camz, 0.0, camy, camz, 0.0, 1.0, 0.0);
			break;

		case GL_TEXTURE_CUBE_MAP_NEGATIVE_Y:
			gluLookAt(camx, camy, camz, camx, 0.0, camz, 0.0, 0.0, -1.0);
			break;
		case GL_TEXTURE_CUBE_MAP_POSITIVE_Y:
			gluLookAt(camx, camy, camz, camx, 0.0, camz, 0.0, 0.0, 1.0);
			break;
		case GL_TEXTURE_CUBE_MAP_NEGATIVE_Z:
			gluLookAt(camx, camy, camz, camx, camy, 0.0, 0.0, 1.0, 0.);
			break;
		case GL_TEXTURE_CUBE_MAP_POSITIVE_Z:
			gluLookAt(camx, camy, camz, camx, camy, 0.0, 0.0, 1.0, 0.);
			break;
		}
		break;
	case M_INSIDE:
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		glFrustum(-ZNEAR, ZNEAR, -ZNEAR, ZNEAR, ZNEAR, ZFAR);
		
#if 1
		{
		/* Recover the current matrix */
		GLfloat projectionMatrix[16];
		glGetFloatv(GL_PROJECTION_MATRIX,projectionMatrix);
		/* Modify the entries */
		projectionMatrix[10]=(-ZFAR-ZNEAR)/(ZFAR-ZNEAR);
		projectionMatrix[14]=(-2*ZFAR*ZNEAR)/(ZFAR-ZNEAR);
		/* Load the modified matrix */
		glLoadMatrixf(projectionMatrix);
		}
#endif

		glViewport(0,0,256,256);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		switch(screen)
		{
		case GL_TEXTURE_CUBE_MAP_POSITIVE_X:
			gluLookAt(0, 0, 0, -1, 0, 0, 0.0, 1.0, 0.0);
			break;
		case GL_TEXTURE_CUBE_MAP_NEGATIVE_X:
			gluLookAt(0, 0, 0, 1, 0, 0, 0.0, 1.0, 0.0);
			break;
		case GL_TEXTURE_CUBE_MAP_POSITIVE_Y:
			gluLookAt(0, 0, 0, 0, -1, 0, 0.0, 0.0, 1.0);
			break;
		case GL_TEXTURE_CUBE_MAP_NEGATIVE_Y:
			gluLookAt(0, 0, 0, 0, 1, 0, 0.0, 0.0, -1.0);
			break;
		case GL_TEXTURE_CUBE_MAP_POSITIVE_Z:
			gluLookAt(0, 0, 0, 0, 0, 1, 0.0, 1.0, 0.0);
			break;
		case GL_TEXTURE_CUBE_MAP_NEGATIVE_Z:
			gluLookAt(0, 0, 0, 0, 0, -1, 0.0, 1.0, 0.0);
			break;
		}
		break;
	}

	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
//	if (cameramode == M_INSIDE);
//	glScalef(-1,-1,-1);
}


void
initCubemap(void)
{
	//RGBA8 Cubemap texture, 24 bit depth texture, 256x256
	glGenTextures(1, &color_tex);
	glBindTexture(GL_TEXTURE_CUBE_MAP, color_tex);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	//NULL means reserve texture memory, but texels are undefined
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X+0, 0, GL_RGBA8, 256, 256, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X+1, 0, GL_RGBA8, 256, 256, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X+2, 0, GL_RGBA8, 256, 256, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X+3, 0, GL_RGBA8, 256, 256, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X+4, 0, GL_RGBA8, 256, 256, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X+5, 0, GL_RGBA8, 256, 256, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);

	//-------------------------
	glGenFramebuffersEXT(1, &fb);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb);

	//Attach one of the faces of the Cubemap texture to this FBO
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_CUBE_MAP_POSITIVE_X, color_tex, 0);
	//-------------------------
	glGenRenderbuffersEXT(1, &depth_rb);
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depth_rb);
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, 256, 256);
	//-------------------------
	//Attach depth buffer to FBO
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, depth_rb);

	//-------------------------
	//Does the GPU support current FBO configuration?
	GLenum status;
	status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
	// status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
	switch(status)
	{
	case GL_FRAMEBUFFER_COMPLETE_EXT:
		printf("good\n");
		break;
	default:
		assert(0);
	}
}


void
renderSceneToCubeMap(void)
{
	GLfloat m[16];

	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depth_rb);
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, depth_rb);

	setRenderSceneToCubemapState();

	if (rotating)
		outsideCamY+=1.0f;

	for (int i=0;i<6;i++)
	{
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_CUBE_MAP_POSITIVE_X+i, color_tex, 0);
#if (COLOUR_BACKGROUND)
		switch(i){ // AJS temp, clear background so can tell which screen is which
			case 0:
				glClearColor(0.7, 0.0, 0.0, 1.0); // +X
				break;
			case 1:
				glClearColor(0.3, 0.0, 0.0, 1.0); // - X
				break;
			case 2:
				glClearColor(0.0, 0.7, 0.0, 1.0);
				break;
			case 3:
				glClearColor(0.0, 0.3, 0.0, 1.0);
				break;
			case 4:
				glClearColor(0.0, 0.0, 0.7, 1.0);
				break;
			case 5:
				glClearColor(0.0, 0.0, 0.3, 1.0);
				break;
		}
#else
		glClearColor(0.0, 0.0, 0.0, 1.0);
#endif

		switch(cameramode)
		{
		case M_INSIDE:
			glClearDepth(0.0f);
			glDepthFunc(GL_GEQUAL);
			break;
		case M_TEST:
		case M_OUTSIDE:
			glClearDepth(1.0f);
			glDepthFunc(GL_LEQUAL);
			break;
		}
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		if (i==5 && point_cloud_enabled)
			setRenderSceneToCubemapMatrices(GL_TEXTURE_CUBE_MAP_POSITIVE_X+i);
		else if (!point_cloud_enabled)
			setRenderSceneToCubemapMatrices(GL_TEXTURE_CUBE_MAP_POSITIVE_X+i);
		glPushMatrix();
		build_rotmatrix(m, curquat);
		glMultMatrixf(&m[0]);
		if (i==5 && point_cloud_enabled)
			renderPointCloud();
		else if (!point_cloud_enabled)
			renderHead();
		//renderBoxes();
		glPopMatrix();
	}
	glClearColor(0.0,0.0,0.0,1.0);
}

void
renderCubemapToSphere(void)
{
	glViewport(0,0,W,H);
	//Bind 0, which means render to back buffer
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	// Ditto for depth buffer
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, 0);
	glClearDepth(1.0f);
	glDepthFunc(GL_LEQUAL);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Now we render the cubemap on the sphere
	setRenderCubemapToSphereState();
	setRenderCubemapToSphereMatrices();

	glPushMatrix();
	glutSolidSphere(1.0, 35, 35);
	glPopMatrix();
}

void
display(void)
{
	//static int z = 0, z2 = 0;
	static char iddata[128];
	static char type[128];
	static char name[128];
	static char my_avatar[128];
	float avatar_info[1024];
	float q[4];
	float h=0.f, p=0.f, r=0.f;
	int c = 0; //head node is 7 for new avatars
	//int p = 2; //pelvis node is 2
	//int r = 0; //root node is 0
	int d = 9; //eye node is 9 for right eye of new avatars
#if (OFFLINE_TEST==0)
	strcpy(name,"");
	strcpy(type,"");
	strcpy(iddata,"");
	strcpy(my_avatar,"");
	checkStatus(iddata,type,name,my_avatar);
	if ((strcmp(type,"AVATAR")==0)||(strcmp(type,"SPECTATOR")==0))
	{
		if (!new_clients[iddata].empty())
			new_clients[iddata].clear();
		new_clients[iddata].push_back(name);
		new_clients[iddata].push_back(type);
		new_clients[iddata].push_back(my_avatar);
		printf("new id=%s type=%s name=%s avatar=%s\n",iddata,type,name,my_avatar);
	} 
	for ( std::map<std::string, std::vector<std::string> >::iterator cIter = new_clients.begin(); cIter!=new_clients.end(); cIter++ )
	{ 
		strcpy(iddata,cIter->first.c_str()); //copy guid
		strcpy(name,cIter->second[0].c_str()); //copy name
		strcpy(type,cIter->second[1].c_str()); //copy type
		strcpy(my_avatar,cIter->second[2].c_str()); //copy avatar name
		if (strcmp(name,visitor)==0) //found visitor
		{
			fetch(iddata,avatar_info);
			//apply rotation to eyes first
			float avatar_arr[3] = {avatar_info[4+(d*8)],avatar_info[5+(d*8)],avatar_info[6+(d*8)]};
			axis_to_quat(avatar_arr,avatar_info[7+(d*8)],q);
			//q[0]=avatar_info[4+(d*8)]; q[1]=avatar_info[5+(d*8)]; q[2]=avatar_info[6+(d*8)]; q[3]=avatar_info[7+(d*8)];
			//halca_setRotation(id,9,q);
			//halca_setRotation(id,11,q);
			//then apply rotation to head
			//printf("%s %.3f %.3f %.3f %.3f\n",name,avatar_info[4+(c*8)],avatar_info[5+(c*8)],avatar_info[6+(c*8)],avatar_info[7+(c*8)]);
			avatar_arr[0]=avatar_info[4+(c*8)]; avatar_arr[1]=avatar_info[5+(c*8)]; avatar_arr[2]=avatar_info[6+(c*8)];
			CMatrix m;
			float m_rot = 0.0f;
			//extract y_angle
			if ((avatar_info[4+(c*8)]==0)&&(avatar_info[5+(c*8)]==1)&&(avatar_info[6+(c*8)]==0)){
				p = avatar_info[7+(c*8)]*RADTODEG;
			} else if ((avatar_info[4+(c*8)]==0)&&(avatar_info[5+(c*8)]==-1)&&(avatar_info[6+(c*8)]==0)) {
				p = -avatar_info[7+(c*8)]*RADTODEG;
			} else { //convert to euler to get y_angle
				m.RotateMatrix(avatar_info[7+(c*8)]*RADTODEG,avatar_info[4+(c*8)],avatar_info[5+(c*8)],avatar_info[6+(c*8)]);
				m.ToEuler(&h,&p,&r);
			}
			//reverse angle if camera mode is inside
			if (cameramode==M_INSIDE)
			{
				p = -p;
				m_rot = -avatar_info[7+(c*8)];
			} else { m_rot = avatar_info[7+(c*8)]; }
			//front view
			if (front_view)
			{
				if (y_rot) {
					rot_angle = p;//-avatar_info[7+(c*8)]*RADTODEG;
					rotx=0.0; roty=0.0; rotz=1.0;
				} else {
					rot_angle = m_rot*RADTODEG;
					rotx=avatar_info[4+(c*8)]; roty=avatar_info[6+(c*8)]; rotz=avatar_info[5+(c*8)];
				}
				avatar_arr[0]=0.0f; avatar_arr[1]=1.0f; avatar_arr[2]=0.0f;
				axis_to_quat(avatar_arr,0.f,curquat);
				//outsideCamY=-90.0f;
				//printf("%.3f\t",rot_angle);
			} else { //other views
				if (y_rot) {
					avatar_arr[0]=0.0f; avatar_arr[1]=1.0f; avatar_arr[2]=0.0f;
					axis_to_quat(avatar_arr,p*DEGTORAD,curquat); 
				}
				else { 
					axis_to_quat(avatar_arr,m_rot,curquat); 
				}
			}
			//lip sync
			jaw_x = avatar_info[1+(13*8)]; //for lip movement - jaw is assumed to be node 13
			continue;
		}

		if ((strcmp(name,local)==0) && (!front_view))//found local for outside_view 'o'
		{
			fetch(iddata,avatar_info);
			//printf("%.3f %.3f %.3f %.3f\n",avatar_info[4+(c*8)],avatar_info[5+(c*8)],avatar_info[6+(c*8)],avatar_info[7+(c*8)]);
			//assumes bone 5 - spine1
			if (auto_frustum)
			{
				// heading vector i.e. straight ahead from sphere position
				CVec3 f = CVec3(0.f,0.f,-1.f);
				f.Normalize();
				// sphere-to-local vector i.e. position of local as sphere is assumed to be close to origin
				//to do - need to add the translation offset of sphere from controlling camera (kinect/ladybug)
				CVec3 aimV = CVec3(avatar_info[1+(5*8)],/*avatar_info[2+(5*8)]*/0.f,avatar_info[3+(5*8)]);
				aimV.Normalize();
				// get rotation angle from heading to aiming vector
				outsideCamY = -90.0f + RAD_TO_DEG(aimV.angle_between(aimV,f));
			}
			continue;
		}
	}
	//for (int c=0; c<count; c++)
	//	printf("object %.f - %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",avatar_info[0+(c*8)],avatar_info[1+(c*8)],avatar_info[2+(c*8)],avatar_info[3+(c*8)],avatar_info[4+(c*8)],avatar_info[5+(c*8)],avatar_info[6+(c*8)],avatar_info[7+(c*8)]);
#endif
#ifdef QSTREAM
	//apply quaternion (or euler) rotation to eyes first
	float avatar_arr[3] = {eye_rx,eye_ry,eye_rz};
	if (eye_rw==899.76) {
		if (replay2)	{
			halca_resetBone(id,9);	//left eye
			halca_addRotationEuler(id,9,avatar_arr);
			halca_resetBone(id,11);	//right eye
			halca_addRotationEuler(id,11,avatar_arr);
		}
		else {
			halca_resetBone(id,9);	//left eye
			halca_setRotationEuler(id,9,avatar_arr);
			halca_resetBone(id,11);	//right eye
			halca_setRotationEuler(id,11,avatar_arr);
		}
	} else {
		//axis_to_quat(avatar_arr,eye_rw,q);	//uncomment if axis-angle
		q[0]=eye_rx; q[1]=eye_ry; q[2]=eye_rz; q[3]=eye_rw;
		if (replay2)	{
			halca_resetBone(id,9);	//left eye
			halca_addRotation(id,9,q);
			halca_resetBone(id,11);	//right eye
			halca_addRotation(id,11,q);
		}
		else {
			halca_resetBone(id,9);	//left eye
			halca_setRotation(id,9,q);
			halca_resetBone(id,11);	//right eye
			halca_setRotation(id,11,q);
		}
	}

	//then apply quaternion rotation to head
	CQuat headq(-head_ry,-head_rx,-head_rz,head_rw);
	if (constrain_head_rot) //head needs constraining
	{
		headq.constrain(DEG_TO_RAD(constrainAngle)); 
		CQuat constrain_headq(-constrain_head_ry,-constrain_head_rx,-constrain_head_rz,constrain_head_rw);
		headq = headq * constrain_headq;
	}
	float head_ax, head_ay, head_az, head_aw;
	headq.getAngleAxis(&head_aw,&head_ax,&head_ay,&head_az);
	avatar_arr[0]=head_ax; avatar_arr[1]=head_ay; avatar_arr[2]=head_az;
	//convert to euler to get y_angle
	CMatrix m;
	m.RotateMatrix(head_aw*RADTODEG,head_ax,head_ay,head_az);
	m.ToEuler(&h,&p,&r);
	float m_rot = 0.0f;
	m_rot = head_aw; 
	//front view
	if (front_view)
	{
		if (y_rot) {
			rot_angle = h;
			rotx=0.0; roty=0.0; rotz=1.0*reverseY;
		} else {
			rot_angle = m_rot*RADTODEG;
			rotx=head_ax; roty=-head_az; rotz=-head_ay;
		}
		avatar_arr[0]=0.0f; avatar_arr[1]=1.0f; avatar_arr[2]=0.0f;
		axis_to_quat(avatar_arr,0.f,curquat);
		outsideCamY=-90.0f;
		//printf("%.3f\t",rot_angle);
	} else { //other views
		if (y_rot) { //y-axis rotation
			avatar_arr[0]=0.0f; avatar_arr[1]=1.0f; avatar_arr[2]=0.0f;
			if (cameramode==M_INSIDE)
				axis_to_quat(avatar_arr,-h*DEGTORAD,curquat); 
			else
				axis_to_quat(avatar_arr,h*DEGTORAD,curquat); 
		}
		else { //arbitrary axis rotation
			if ((cameramode==M_OUTSIDE)&&(!front_view))	{
				avatar_arr[0]=-head_ax; avatar_arr[1]=-head_ay; avatar_arr[2]=-head_az;
			}
			axis_to_quat(avatar_arr,m_rot,curquat); 
		}
	}

	if ((auto_frustum)&&(!front_view))
	{
		// heading vector i.e. straight ahead from sphere position
		CVec3 f = CVec3(0.f,0.f,-1.f);
		f.Normalize();
		// sphere-to-local vector i.e. position of local as sphere is assumed to be close to origin
		// subtract the translation offsets, x and z of sphere from controlling camera (kinect/ladybug) 
		CVec3 aimV = CVec3(local_x-offset_x,/*avatar_info[2+(5*8)]*/0.f,local_z-offset_z);
		if ((local_x==0.f)&&(local_z==0.f))
			aimV = CVec3(0.f,0.f,1.f); //reset to straight ahead
		aimV.Normalize();
		// get rotation angle from heading to aiming vector
		if ((local_x-offset_x)<0.f)
			outsideCamY = -90.0f + (RAD_TO_DEG(aimV.angle_between(f,aimV))* -1.f);
		else
			outsideCamY = -90.0f + RAD_TO_DEG(aimV.angle_between(f,aimV));
	}
#endif
	glClearColor(1, 1, 1, 1);
	renderSceneToCubeMap();
	renderCubemapToSphere();
	glutSwapBuffers();
}

void
reshape(int w, int h)
{
	W = w;
	H = h;
}


void
animate(void)
{
#if (OFFLINE_TEST==0)
	check();
#endif
#ifdef QSTREAM
	QBuffer buffer;												//read shared memory
	QDataStream in(&buffer);
	clientMemory.lock();										//lock
	buffer.setData((char*)clientMemory.constData(), clientMemory.size());
	buffer.open(QBuffer::ReadOnly);
	static QString message;
	QString newmessage;
	in >> newmessage;
	clientMemory.unlock();										//unlock
	//std::cout << newmessage.toStdString() << std::endl;
	if (message != newmessage)
	{
		message = newmessage;
		if ( message == "quit" ) {
			exit(0); //exit this program
		}
		QStringList appList = message.split(";");		//split on ;
		//0 - camera mode
		if (appList[0]=="TEST") {
			cameramode = M_TEST;
			front_view = false;
		} else if (appList[0]=="INSIDE") {
			cameramode = M_INSIDE;
			front_view = false;
		} else if (appList[0]=="OUTSIDE") {
			cameramode = M_OUTSIDE;
			front_view = false;
		} else if (appList[0]=="FRONT") {
			cameramode = M_OUTSIDE;
			front_view = true;
		}
		//1 - visitor name
		strcpy(visitor,appList[1].toStdString().c_str());
		//2 - visitor_headnode
		visitor_headnode = appList[2].toInt();
		//3 - local name
		strcpy(local,appList[3].toStdString().c_str());
		//4 - local_headnode
		local_headnode = appList[4].toInt();
		//5 - translateX
		translate_x = appList[5].toFloat();
		//6 - translateY
		translate_y = appList[6].toFloat();
		//7 - translateZ
		translate_z = appList[7].toFloat();
		//8 - scale_inside
		scale_inside = appList[8].toFloat();
		//9 - scale_outside
		scale_outside = appList[9].toFloat();
		//10 - rotation: "y_axis" or "arbitrary"
		if (appList[10]=="y-axis")
			y_rot = true;
		else if (appList[10]=="arbitrary")
			y_rot = false;
		//11 - inside_headY
		inside_headY = appList[11].toFloat();
		//12 - inside_sphereY
		inside_sphereY = appList[12].toFloat();
		//13 - inside_headY
		outside_headY = appList[13].toFloat();
		//14 - inside_sphereY
		outside_sphereY = appList[14].toFloat();
		//15 - front_headY
		front_headY = appList[15].toFloat();
		//16 - front_sphereY
		front_sphereY = appList[16].toFloat();
		//17 - visitor eye rotation x
		eye_rx = appList[17].toFloat();
		//18 - visitor eye rotation y
		eye_ry = appList[18].toFloat();
		//19 - visitor eye rotation z 
		eye_rz = appList[19].toFloat();
		//20 - visitor eye rotation w 
		eye_rw = appList[20].toFloat();
		//21 - visitor head position x
		head_x = appList[21].toFloat();
		//22 - visitor head position y 
		head_y = appList[22].toFloat();
		//23 - visitor head position z 
		head_z = appList[23].toFloat();
		//24 - visitor head rotation x
		head_rx = appList[24].toFloat();
		//25 - visitor head rotation y 
		head_ry = appList[25].toFloat();
		//26 - visitor head rotation z 
		head_rz = appList[26].toFloat();
		//27 - visitor head rotation w 
		head_rw = appList[27].toFloat();
		//28 - local head position x
		local_x = appList[28].toFloat();
		//29 - local head position y
		local_y = appList[29].toFloat();
		//30 - local head position z
		local_z = appList[30].toFloat();
		//31 - local head rotation x
		local_rx = appList[31].toFloat();
		//32 - local head rotation y
		local_ry = appList[32].toFloat();
		//33 - local head rotation z
		local_rz = appList[33].toFloat();
		//34 - local head rotation w
		local_rw = appList[34].toFloat();
		//35 - jaw [lip sync]
		jaw_x = appList[35].toFloat();
		//36 - reverseX
		reverseX = appList[36].toInt();
		head_rx *= reverseX;
		//37 - reverseY
		reverseY = appList[37].toInt();
		head_ry *= reverseY;
		//38 - reverseZ
		reverseZ = appList[38].toInt();
		head_rz *= reverseZ;
		//39 - avatar_head cfg file
		if (avatarCFG != appList[39]) //avatar changed
		{
			avatarCFG = appList[39]; //set as new cfg
			float arr[3] = {100.f,100.f,100.f};
			halca_setTranslation(id,0,arr); //move the avatar's head away from view
			if (avatars.find(avatarCFG.toStdString())!=avatars.end()) //if already loaded
			{
				id = avatars[avatarCFG.toStdString()];// set new avatar as current id
			} else { //need to add new character
				char cfg[128]; strcpy(cfg, avatarCFG.toStdString().c_str());
				char av[128]; strcpy(av,cfg); strcat(av,".cfg");
				id = halca_addCharacter(cfg,av);
				avatars[avatarCFG.toStdString()] = id;
			}
		}
		//40 - sphere offset x from origin
		offset_x = appList[40].toFloat();
		//41 - sphere offset y from origin
		offset_y = appList[41].toFloat();
		//42 - sphere offset z from origin 
		offset_z = appList[42].toFloat();
		//43 - constrain head rotation
		if (constrain_head_rot != appList[43].toInt())
		{
			constrain_head_rot = appList[43].toInt();
			//if constrain head rot is checked, copy current head rot
			constrain_head_rx=head_rx;
			constrain_head_ry=head_ry;
			constrain_head_rz=head_rz;
			constrain_head_rw=head_rw;
		}
		//44 - constrainAngle
		constrainAngle = appList[44].toFloat();
		//45 - 145 (Face Tracking)
		int i=45;
		char tmp[128];
		while ( i < (appList.size()-3))
		{
			int jid = appList[i].toInt(); i++;
			allbones_pos[jid*3] = appList[i].toFloat(); i++;
			allbones_pos[jid*3+1] = appList[i].toFloat(); i++;
			allbones_pos[jid*3+2] = appList[i].toFloat(); i++;
			//sprintf(tmp,"%d - %.4f,%.4f,%.4f",jid,allbones_pos[jid*3],allbones_pos[jid*3+1],allbones_pos[jid*3+2]);
			//std::cout << tmp << std::endl;
		}
	}

#endif
	if (spinning)
		add_quats(lastquat, curquat, curquat);
	glutPostRedisplay();

}

void
special(int key, int x, int y)
{
	static int a1=0,b1=0,a2=0,b2=0,a3=0,b3=0;
	switch(key)
	{
	case GLUT_KEY_LEFT:
		if ((cameramode==M_OUTSIDE)&&(!front_view))	{
			a1+=1;
			outside_headY = 90.f * (a1 % 4);
		}
		else if ((cameramode==M_OUTSIDE)&&(front_view))	{
			a2+=1;
			front_headY = 90.f * (a2 % 4);
		}
		else if (cameramode==M_INSIDE) {
			a3+=1;
			inside_headY = 90.f * (a3 % 4);
		}
		break;
	case GLUT_KEY_RIGHT:
		if ((cameramode==M_OUTSIDE)&&(!front_view))	{
			a1-=1;
			outside_headY = 90.f * (a1 % 4);
		}
		else if ((cameramode==M_OUTSIDE)&&(front_view))	{
			a2-=1;
			front_headY = 90.f * (a2 % 4);
		}
		else if (cameramode==M_INSIDE) {
			a3-=1;
			inside_headY = 90.f * (a3 % 4);
		}
		break;
	case GLUT_KEY_UP:
		if ((cameramode==M_OUTSIDE)&&(!front_view))	{
			b1+=1;
			outside_sphereY = 90.f * (b1 % 4);
		}
		else if ((cameramode==M_OUTSIDE)&&(front_view))	{
			b2+=1;
			front_sphereY = 90.f * (b2 % 4);
		}
		else if (cameramode==M_INSIDE) {
			b3+=1;
			inside_sphereY = 90.f * (b3 % 4);
		}
		break;
	case GLUT_KEY_DOWN:
		if ((cameramode==M_OUTSIDE)&&(!front_view))	{
			b1-=1;
			outside_sphereY = 90.f * (b1 % 4);
		}
		else if ((cameramode==M_OUTSIDE)&&(front_view))	{
			b2-=1;
			front_sphereY = 90.f * (b2 % 4);
		}
		else if (cameramode==M_INSIDE) {
			b3-=1;
			inside_sphereY = 90.f * (b3 % 4);
		}
		break;
	}
}

void
keyboard(unsigned char c, int x, int y)
{
	switch (c) {
		case 27:  //Esc
		case 'Q':
		case 'q':
		#if (OFFLINE_TEST==0)
		  removeAllNodes();
		  stop();
		#endif
		  exit(0);
		  break;
		case ' ':
		  rotating = 1 - rotating;
		  break;
		case 'x':
		  translate_x = translate_x + 0.01f;
		  break;
		case 'X':
		  translate_x = translate_x - 0.01f;
		  break;
		case 'y':
		  translate_y = translate_y + 0.01f;
		  break;
		case 'Y':
			translate_y = translate_y - 0.01f;
		  break;
		case 'z':
			translate_z = translate_z + 0.01f;
			zoom = zoom + 0.5f;
			printf("%.2f ",zoom);
		  break;
		case 'Z':
			translate_z = translate_z - 0.01f;
			zoom = zoom - 0.5f;
			printf("%.2f ",zoom);
			break;
		case '-':
		  scale_outside *=0.75;
		  scale_inside *=0.75;
		  break;
		case '+':
		  scale_outside *=1.2;
		  scale_inside *=1.2;
		  break;
		case 'i':
		  cameramode = M_INSIDE;
		  front_view = false;
		  glutPostRedisplay();
		  break;
		case 'o':
		  cameramode = M_OUTSIDE;
		  front_view = false;
		  glutPostRedisplay();
		  break;
		case 'O':
		  cameramode = M_OUTSIDE;
		  front_view = true;
		  glutPostRedisplay();
		  break;
		case 'a':
		  rot_angle += 1.f;
		  //printf("%.2f\t",rot_angle);
		  glutPostRedisplay();
		  break;
		case 'A':
		  rot_angle -= 1.f;
		  //printf("%.2f\t",rot_angle);
		  glutPostRedisplay();
		  break;
		case 't':
		  if (!auto_frustum)
		  {
			  outsideCamY += 13.7f;
			  //printf("%.2f\t",rot_angle);
			  glutPostRedisplay();
		  }
		  break;
		case 'T':
		  if (!auto_frustum)
		  {
			  outsideCamY -= 13.7f;
			  //printf("%.2f\t",rot_angle);
			  glutPostRedisplay();
		  }
		  break;
		case 'd':
			auto_frustum = !auto_frustum;
			if (auto_frustum)
				outsideCamY = -90.f;
			printf("Frustum control is  %s\n", (auto_frustum)?"automated":"manual - press 't' or 'T' (experiment control)");
		  break;
		case '2':
			y_rot = !y_rot;
			printf("enabling rotation on %s\n", (y_rot)?"y-axis only":"arbitrary axis");
		  break;
		case 'r':
			replay1 = !replay1;
			printf("%s mode - to correct jaw\n", (replay1)?"replay":"live");
		  break;
		case 'R':
			replay2 = !replay2;
			printf("%s mode - to correct eyes\n", (replay2)?"replay":"live");
		  break;
		case 'k':
			if (!point_cloud_enabled) {
				kin = new Kinect();
				kin_running = kin->start();
				if (kin_running)
				{
					kin->alignViewPoint();
					printf("Kinect started\n");
				}	
				gpc.width = 640;
				gpc.height = 480;
				gpc.points.resize(gpc.width*gpc.height);
				cameramode=M_OUTSIDE;
				point_cloud_enabled = true;
			} else {
				delete kin;
				point_cloud_enabled = false;
			}
		  break;
		case 'p':
			max_dist+=0.01f;
		  break;
		case 'P':
			max_dist-=0.01f;
		  break;
	}
}

void
menu(int item)
{
	switch (item) {
  case M_INSIDE:
  case M_OUTSIDE:
  case M_TEST:
	  cameramode=item;
	  break;
  default:
	  assert(0);
	}
	glutPostRedisplay();
}

void
mouse(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		spinning = 0;
		moving = 1;
		beginx = x;
		beginy = y;
	}
	if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
		moving = 0;
		spinning = 0;
	}
}

void
motion(int x, int y)
{
	if (moving) {
		trackball(lastquat,
			(2.0 * beginx - W) / W,
			(H - 2.0 * beginy) / H,
			(2.0 * x - W) / W,
			(H - 2.0 * y) / H
			);
		beginx = x;
		beginy = y;
		spinning = 1;
	}
}

/*void
cleanup(void)
{
	removeAllNodes();
	stop();
}*/

int
main(int argc, char **argv)
{
	int i;

	printf("Press Esc to quit ...\n");

#ifdef QSTREAM
	//QApplication app(argc, argv, false);
	clientMemory.setKey("sphereDisplay");											//with key
	if (!clientMemory.attach())												//attach
	{
		printf("Unable to attach to shared memory segment");				//inform if unsuccessful
		return 0;															//return
	}
	//get avatar name and halca init dir
	strcpy(halca_dir,argv[1]);
	strcpy(which_avatar,argv[2]);
#endif

#if (OFFLINE_TEST==0)
	int connected = startclient("127.0.0.1",/*"128.16.6.145","161.116.7.178","128.16.7.66","128.16.13.93",*/ 12349,"Sphere","SPECTATOR","m016_head.cfg",true);
#endif

#if defined(_WIN32)
	Sleep(100);
#else
	sleep(2);
#endif

#if (OFFLINE_TEST==0)
	printf("client is %s\n", (connected)?"connected":"not connected");
#endif

	glutInitWindowSize(1024, 768);
	glutInit(&argc, argv);
	for (i=1; i<argc; i++) {
		if (!strcmp(argv[i], "-v")) {
			i++;
			strcpy(visitor,argv[i]);
		}
		if (!strcmp(argv[i], "-l")) {
			i++;
			strcpy(local,argv[i]);
		}
	}
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow("Spherical Display App");

	if (!glutExtensionSupported("GL_EXT_texture_cube_map")) {
		printf("cm_demo: Your OpenGL implementation does not support EXT_texture_cube_map.\n");
		printf("cm_demo: This program requires it to run.\n");
		exit(1);
	}

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutSpecialFunc(special);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(animate);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);

	initHALCA();

	initCubemap();
	trackball(curquat, 0.0, 0.0, 0.0, 0.0);

	glutCreateMenu(menu);
	glutAddMenuEntry("Test View", M_TEST);
	glutAddMenuEntry("Inside View", M_INSIDE);
	glutAddMenuEntry("Outside View", M_OUTSIDE);

	glutAttachMenu(GLUT_RIGHT_BUTTON);

//	atexit(cleanup);

	glutMainLoop();
	return 0;
}

