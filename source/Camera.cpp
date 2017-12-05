/*
 *      Camera.cpp
 *
 *      Fabrizio Pece <f.pece@cs.ucl.ac.uk>
 *		University College London
 *		
 *		2011
 *
 *		Camera utilities - Base class for other Cameras classes
 */

#include "Camera.h"

Camera::Camera(){

	state = false;
	width = 200;
	height = 200;

}

Camera::~Camera(){

}

bool Camera::getState(){
	return state;
}

void Camera::setState( bool val){

	state = val;

}
