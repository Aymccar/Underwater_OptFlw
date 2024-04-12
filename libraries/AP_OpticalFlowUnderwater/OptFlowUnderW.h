#pragma once
#include <opencv2/videoio.hpp>
using namspace cv;


class OptFlowUnderWtr {

	public :
		int[3] getDisplacement();
		void updateDisplacement();

		void Connect_camera();
		bool isConnected();	


	private :
		
		VideoCapture cap;
		
		bool connected = false;
		int[3] displacement;

				
		Mat c_frame; //Current frame

		Mat c_pp_frame; //Current pre processed frame
		Mat p_pp_frame; //Previous pre processed frame

		Mat c_OF_frame; //Current frame after opticalflow

		Mat c_PP_frame; //Current frame post processed
		
		void speed_calculation();

		void preProcess();
		void optFlow();
		void postProcess();



}

		
