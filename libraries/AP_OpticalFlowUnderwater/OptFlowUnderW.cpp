#include <OptFlowUnderW.h>

void OptFlowUnderWtr::Connect_camera(){
	cap.open("/dev/video69", CAP_V4L2);
	connected = cap.isOpen();
}


bool OptFlowUnderWtr::isConnected(){
       return connected;}
       

int[3] OptFlowUnderWtr::getDisplacement(){
	return displacement;}

void OptFlowUnderWtr::updateDisplacement(){
	
	if (connected){return;} //I fnot connected, kill

	if (cap.read(c_frame)){ //Try to cath new frame, if it's not the case do something
		//I see nothing I should write it somewhere
	}
	
	p_pp_frame = c_pp_frame; //Current preprocess frame become the previous one

	
	preProcess(); //Pre process the current frame
	optFlow(); //Apply optical flow between previous and current frame
	postProcess(); //Post process the optFlow result

	speed_calculation(); //calculate the speed of the ROV
	return;
}


