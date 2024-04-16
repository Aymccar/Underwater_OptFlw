#include "OptFlowUnderWtr.h"

inline void OptFlowUnderWtr::Connect_camera() {
	cap.open("/dev/video69", CAP_V4L2);
	connected = cap.isOpened();

	if (connected) {
            // Read and set the first two frames as the current frame
    	    Mat frame1, frame2;
    	    cap >> frame1; // Read the first frame



        // Get the dimensions of the image
        int image_width = frame1.cols;
        int image_height = frame1.rows;

        // Calculate the center coordinates of the image
        int center_x = image_width / 2;
        int center_y = image_height / 2;

        // Define the side length of the square
        int side_length_x = 800; // Choose the smaller dimension
        int side_length_y = 400;

        // Calculate the coordinates of the top-left and bottom-right corners of the square
        int x1 = center_x - side_length_x / 2;
        int y1 = center_y - side_length_y / 2;

        // Define the region of interest (ROI)
        Rect roi(x1, y1, side_length_x, side_length_y); // (x, y, width, height)
        // Define the region of interest (ROI)
        //Rect roi(1000, 400, 700, 300); // (x, y, width, height)

	
	}


inline bool OptFlowUnderWtr::isConnected() {
	return connected;
}


inline std::array<float, 3> OptFlowUnderWtr::getDisplacement() {
	return std::array<float, 3>{displacement[0], displacement[1], displacement[2]};
}

inline void OptFlowUnderWtr::updateDisplacement() {

	if (!connected) { 
		std::cout << "Camara not connected." << std::endl;
		return; 
	} //I fnot connected, kill

	//if (!cap.read(c_frame)) { //Try to cath new frame, if it's not the case do something

	//	std::cerr << "Failed to read frame from camera." << std::endl;
	//}




	preProcess(); //Pre process the current frame
	optFlow(); //Apply optical flow between previous and current frame
	postProcess(); //Post process the optFlow result

	//p_pp_frame = c_pp_frame; //Current preprocess frame become the previous one
	return;
}




inline void OptFlowUnderWtr::Cropimage() {
	// Get the dimensions of the image
	int image_width = c_frame.cols;
	int image_height = c_frame.rows;

	// Calculate the center coordinates of the image
	int center_x = image_width / 2;
	int center_y = image_height / 2;

	// Define the side length of the square
	int side_length_x = 800; // Choose the smaller dimension
	int side_length_y = 400;

	// Calculate the coordinates of the top-left and bottom-right corners of the square
	int x1 = center_x - side_length_x / 2;
	int y1 = center_y - side_length_y / 2;

	// Define the region of interest (ROI)
	Rect roi(x1, y1, side_length_x, side_length_y); // (x, y, width, height)
	//Rect roi(1000, 400, 700, 300);
	// Crop the images
	c_pp_frame = c_frame(roi);
	//show image
	//imshow("Cropped Image", c_pp_frame);
	//waitKey(0);
}

inline void OptFlowUnderWtr::preProcess() {
	Cropimage();
	// Extract green channel
	Mat green_channel;
	extractChannel(c_pp_frame, green_channel, 1); // 1 means green channel
	c_pp_frame = green_channel;

}

inline void OptFlowUnderWtr::optFlow() {
	calcOpticalFlowFarneback(p_pp_frame, c_pp_frame, c_OF_frame, 0.8, 3, 20, 5, 7, 1.5, 0);
}

inline void OptFlowUnderWtr::postProcess() {
	// Calculate the size of the optical flow vector field
	std::vector<Mat> flow_channels;
	split(c_OF_frame, flow_channels);
	Mat flow_magnitude;
	magnitude(flow_channels[0], flow_channels[1], flow_magnitude);//Calculate the magnitude of the optical flow vector field
	// Convert to single channel
	flow_magnitude.convertTo(flow_magnitude, CV_32F);
	//std::cout << "Minimum value of flow_magnitude: " << flow_magnitude.dims << std::endl;

	Mat binary_mask = binaryMask(flow_magnitude);
	Mat SelectedaeraMask = Selected_aera_Mask(binary_mask);
	Mat selected_mask = binary_mask.mul(SelectedaeraMask);//Calculate masks
	Mat filtered_flow_magnitude;
	flow_magnitude.copyTo(filtered_flow_magnitude, selected_mask);

	double min_val, max_val;
	minMaxLoc(flow_channels[0], &min_val, &max_val);

	//std::cout << "Minimum value of x: " << min_val << std::endl;
	//std::cout << "Maximum value of y: " << max_val << std::endl;

	//Mat filtered_flow_magnitude = Mat::zeros(flow_magnitude.size(), flow_magnitude.type());
	//flow_magnitude.copyTo(filtered_flow_magnitude, selected_mask);//Filter magnitude

	std::vector<float> non_zero_flow_x, non_zero_flow_y;

	applyMask(flow_channels[0], flow_channels[1], selected_mask, non_zero_flow_x, non_zero_flow_y);

	double min_value_x, max_value_x;
	minMaxLoc(non_zero_flow_x, &min_value_x, &max_value_x);
	//std::cout << "Minimum value of flow_magnitude: " << min_value_x << std::endl;
	//std::cout << "Minimum value of flow_magnitude: " << max_value_x << std::endl;

	//double min_val, max_val;
	//minMaxLoc(non_zero_flow_y, & min_val, & max_val);
	//std::cout << "Minimum value of flow_magnitude_y: " << min_val << std::endl;
	//std::cout << "Minimum value of flow_magnitude_y: " << max_val << std::endl;


	std::vector<float> bin_edges_x, bin_edges_y;
	calculateHistogram(non_zero_flow_x, non_zero_flow_y, bin_edges_x, bin_edges_y);
	//std::cout << "bin_edges_x: ";
	//for (size_t i = 0; i < non_zero_flow_x.size(); ++i) {
	//	std::cout << bin_edges_x[i] << " ";
	//}
	//std::cout << std::endl;
	speed_calculation(bin_edges_x, bin_edges_y, non_zero_flow_x, non_zero_flow_y); //calculate the speed of the ROV

}



inline void OptFlowUnderWtr::speed_calculation(std::vector<float>& bin_edges_x, std::vector<float>& bin_edges_y, std::vector<float>& flow_x, std::vector<float>& flow_y) {
	// Specify interval boundaries
	float lower_bound_x = bin_edges_x[8];
	float upper_bound_x = bin_edges_x[43];

	float lower_bound_y = bin_edges_y[8];
	float upper_bound_y = bin_edges_y[43];

	std::vector<int> target_data_indices_x;
	// Calculate weighted average of data points in the specified interval for x axis
	for (int i = 0; i < flow_x.size(); ++i) {
		if (flow_x[i] >= lower_bound_x && flow_x[i] <= upper_bound_x) {
			target_data_indices_x.push_back(i);
		}
	}


	double sum_x = 0.0;
	for (int idx : target_data_indices_x) {
		sum_x += flow_x[idx];
	}
	double target_data_weighted_mean_x = (target_data_indices_x.size() > 0) ? sum_x / target_data_indices_x.size() : 0.0;


	// Calculate weighted average of data points in the specified interval for y axis
	std::vector<int> target_data_indices_y;
	for (int i = 0; i < flow_y.size(); ++i) {
		if (flow_y[i] >= lower_bound_y && flow_y[i] <= upper_bound_y) {
			target_data_indices_y.push_back(i);
		}
	}

	double sum_y = 0.0;
	for (int idx : target_data_indices_y) {
		sum_y += flow_y[idx];
	}
	double target_data_weighted_mean_y = (target_data_indices_y.size() > 0) ? sum_y / target_data_indices_y.size() : 0.0;

	displacement[0] = target_data_weighted_mean_x;
	displacement[1] = target_data_weighted_mean_y;

	std::cout << "average of optical flow vectors :(" << displacement[0] << "," << displacement[1] << "," << displacement[2] << ")" << std::endl;

}
