#pragma once
#include <opencv2/videoio.hpp>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;


class OptFlowUnderWtr {

public:
	std::array<float, 3> getDisplacement();
	void updateDisplacement();

	void Connect_camera();
	bool isConnected();
	Mat c_frame; //Current frame
	Mat p_pp_frame; //Previous pre processed frame
	Mat c_pp_frame; //Current pre processed frame
	std::array<float, 3> displacement;



private:

	VideoCapture cap;

	bool connected = false;
	





	Mat c_OF_frame; //Current frame after opticalflow

	Mat c_PP_frame; //Current frame post processed


	void speed_calculation(std::vector<float>& bin_edges_x, std::vector<float>& bin_edges_y, std::vector<float>& flow_channels_x, std::vector<float>& flow_channels_y);

	void Cropimage();

	void preProcess();

	void optFlow();

	void postProcess();

	//void binaryMask(flow_magnitude);



};

inline Mat binaryMask(Mat flow_magnitude) {    // Define the threshold value
	double threshold = 1;

	// Create a boolean mask
	Mat bool_mask = flow_magnitude > threshold;

	// Convert boolean mask to an array of integers
	Mat binary_mask;
	// Convert the  boolean mask to an integer-type image, where pixel values of false become 0 and true become 255
	bool_mask.convertTo(binary_mask, CV_8U);

	// Normalize the binary mask
	//binary_mask /= 255; // Divide by 255 to normalize
		// 显示裁剪后的图像

	return binary_mask;
}

inline Mat Selected_aera_Mask(Mat binary_mask) {
	// Define the number and direction of divided rectangular areas
	int num_rows = 8;
	int num_cols = 6;
	int rows = binary_mask.rows;
	int cols = binary_mask.cols;
	int region_height = rows / num_rows;
	int region_width = cols / num_cols;

	// Calculating the proportion of black pixels in each region
	std::vector<double> black_ratios;
	for (int i = 0; i < num_rows; ++i) {
		for (int j = 0; j < num_cols; ++j) {
			Mat region = binary_mask(Rect(j * region_width, i * region_height, region_width, region_height));
	
			int black_pixels = countNonZero(region);
			double total_pixels = region_width * region_height;
			double black_ratio = black_pixels / total_pixels;
			black_ratios.push_back(black_ratio);
		}
	}

	// Find areas with a black pixel ratio greater than 0.2
	double threshold_black_ratio = 0.1;
	std::vector<int> selected_regions;
	for (int i = 0; i < black_ratios.size(); ++i) {
		if (black_ratios[i] >= threshold_black_ratio) {
			selected_regions.push_back(i);
		}
	}

	// Create a blank image for drawing the selected area
	Mat selected_area_mask = Mat::zeros(rows, cols, CV_8U);

	// Marks the selected area on the image
	for (int idx : selected_regions) {
		int i = idx / num_cols;
		int j = idx % num_cols;
		Rect roi(j * region_width,i * region_height, region_width, region_height);//not sure
		selected_area_mask(roi) = 255;
	}
	// Normalize the selected_area_mask
	//selected_area_mask /= 255; // Divide by 255 to normalize

	return selected_area_mask;
}

inline void applyMask(Mat& flow_x1, Mat& flow_y1, Mat& selected_mask, std::vector<float> & non_zero_flow_x, std::vector<float>& non_zero_flow_y) {
	// Use masks to filter the optical flow vector field
	//Mat filtered_flow = flow.mul(selected_mask);


	// 分割光流场的各个通道
	//flow_x = flow_x.mul(selected_mask);
	Mat flowx = Mat::zeros(flow_x1.size(), flow_x1.type());
	Mat flowy = Mat::zeros(flow_y1.size(), flow_y1.type());
	for (int i = 0; i < flow_x1.rows; ++i) {
		for (int j = 0; j < flow_x1.cols; ++j) {
			//std::cout << static_cast<int>(selected_mask.at<uchar>(i, j)) << std::endl;
			if (static_cast<int>(selected_mask.at<uchar>(i, j)) == 255) {
				flowx.at<float>(i, j) = flow_x1.at<float>(i, j);
			}
		}
	}









	
	for (int i = 0; i < flow_y1.rows; ++i) {
		for (int j = 0; j < flow_y1.cols; ++j) {
			//std::cout << static_cast<int>(selected_mask.at<uchar>(i, j)) << std::endl;
			if (static_cast<int>(selected_mask.at<uchar>(i, j)) == 255) {
				flowy.at<float>(i, j) = flow_y1.at<float>(i, j);
			}
		}
	}
	


	// Find non-zero flow vectors
	//std::vector<std::vector<int>> non_zero_flow;
	//std::vector<float> non_zero_flow_x, non_zero_flow_y;
	for (int i = 0; i < flowx.rows; ++i) {
		for (int j = 0; j < flowx.cols; ++j) {
			if (flowx.at<float>(i, j)!= 0 && flowy.at<float>(i, j) != 0) {
				// 创建一个包含两个值的二维向量，并将其添加到non_zero_flow中
				//std::vector<float> flow_vector = { flow_x.at<float>(i, j), flow_y.at<float>(i, j) };

				non_zero_flow_x.push_back(flowx.at<float>(i, j));
				non_zero_flow_y.push_back(flowy.at<float>(i, j));
			}
		}
	}
	//std::cout << "Non-zero count in flowxnm: " << non_zero_flow_x.size() << std::endl;

	double min_valeur, max_valeur;

	cv::minMaxLoc(non_zero_flow_x, &min_valeur, &max_valeur);
	//std::cout << "Minimum value of flow__x: " << min_valeur << std::endl;
	//std::cout << "Minimum value of flow__x: " << max_valeur << std::endl;

	double min_val, max_val;
	cv::minMaxLoc(non_zero_flow_y, &min_val, &max_val);
	//std::cout << "Minimum value of flow__y: " << min_val << std::endl;
	//std::cout << "Minimum value of flow__y: " << max_val << std::endl;

	//bitwise_and((cv::abs(flow_x) > 0), (cv::abs(flow_y) > 0), non_zero_flow);

	//minMaxLoc(flow_x, &min_x, &max_x);

	
}

inline void calculateHistogram(std::vector<float>& non_zero_flow_x, std::vector<float>& non_zero_flow_y, std::vector<float>& bin_edges_x, std::vector<float>& bin_edges_y) {

	int num_bins = 50;
	// Calculate the minimum and maximum values of flow_x

	Mat Flow_x(non_zero_flow_x);
	Mat Flow_y(non_zero_flow_y);
	double min_value_x, max_value_x;
	minMaxLoc(Flow_x, &min_value_x, &max_value_x);

	// Set the range for histogram based on the minimum and maximum values of flow_x
	float range_x[] = { static_cast<float>(min_value_x), static_cast<float>(max_value_x) };

	Mat hist_x, hist_y;

	int channels_x[] = { 0 }; // Channel index

	// Convert min_value_x and max_value_x to float type
	float min_value_x_float = static_cast<float>(min_value_x);
	float max_value_x_float = static_cast<float>(max_value_x);
	const float* hist_ranges_x[] = { &min_value_x_float, &max_value_x_float };
	cv::calcHist(&Flow_x, 1, channels_x, Mat(), hist_x, 1, &num_bins, hist_ranges_x, true, false);


	// Extract bin edges for the y component
	for (int i = 0; i <= num_bins; ++i) {
		bin_edges_x.push_back(range_x[0] + (i * (range_x[1] - range_x[0])) / num_bins);
	}
	
	//std::cout << "bin_edges_x: "<< min_value_x << max_value_x << std::endl;



	// Set the range for histogram based on the minimum and maximum values of flow_y
	double min_value_y, max_value_y;
	minMaxLoc(Flow_y, &min_value_y, &max_value_y);
	float range_y[] = { static_cast<float>(min_value_y), static_cast<float>(max_value_y) };

	float min_value_y_float = static_cast<float>(min_value_y);
	float max_value_y_float = static_cast<float>(max_value_y);
	const float* hist_ranges_y[] = { &min_value_y_float, &max_value_y_float };
	// Calculate histogram of the y-component
	int channels_y[] = { 0 }; // Channel index
	cv::calcHist(&Flow_y, 1, channels_y, Mat(), hist_y, 1, &num_bins, hist_ranges_y, true, false);

	// Extract bin edges for the y component
	for (int i = 0; i <= num_bins; ++i) {
		bin_edges_y.push_back(range_y[0] + (i * (range_y[1] - range_y[0])) / num_bins);
	}

}



