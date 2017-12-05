/*
 *      Camera.h
 *
 *      Fabrizio Pece <f.pece@cs.ucl.ac.uk>
 *		University College London
 *		
 *		2011
 *
 *		HEADER - Camera utilities - Base class for other Cameras classes
 */

#ifndef CAMERA_H_
#define CAMERA_H_


#include <stdio.h>
#include <string.h>

class Camera{

public:

	Camera();
	~Camera();

	virtual const unsigned char* grabRGB() { const unsigned char* dummy; return dummy;};
	virtual bool start() { setState(true); return false; };
	virtual bool stop() { setState(false); return false; };
	bool saveImage(char* filename);


	bool getState();
	int getWidth() { return width; };
	int getHeight() { return height; };

	void setState(bool value);


	int width;
	int height;

private:

	bool state;

};

#endif //CAMERA_H_
