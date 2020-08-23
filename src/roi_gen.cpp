//test findCountours; building up to camera calibration
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//TODO Why are these deps listed twice?
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"

#include <boost/filesystem.hpp>

int u1;
int v1;
int u2;
int v2;
int cc;

cv::Mat img;
cv::Mat img_pure;

std::string full_code;

void click_cb(int event, int x, int y, int flags, void* param){
	if(event == cv::EVENT_LBUTTONDOWN){
		cc++;
		if(cc > 2){
			cc = 0;
			img_pure.copyTo(img);
			cv::imshow(full_code, img);
			return;
		}
		if(cc == 1){
			u1 = x;
			v1 = y;
			img.at<uchar>(v1 - 1,	u1 - 1,	0) = 255;
			img.at<uchar>(v1 - 1,	u1,	0) = 255;
			img.at<uchar>(v1 - 1,	u1 + 1,	0) = 255;
			img.at<uchar>(v1,	u1 - 1,	0) = 255;
			img.at<uchar>(v1,	u1 + 1,	0) = 255;
			img.at<uchar>(v1 + 1,	u1 - 1,	0) = 255;
			img.at<uchar>(v1 + 1,	u1,	0) = 255;
			img.at<uchar>(v1 + 1,	u1 + 1,	0) = 255;
			cv::imshow(full_code, img);
		}
		if(cc == 2){
			u2 = x;
			v2 = y;
			img.at<uchar>(v2 - 1,	u2 - 1,	0) = 255;
			img.at<uchar>(v2 - 1,	u2,	0) = 255;
			img.at<uchar>(v2 - 1,	u2 + 1,	0) = 255;
			img.at<uchar>(v2,	u2 - 1,	0) = 255;
			img.at<uchar>(v2,	u2 + 1,	0) = 255;
			img.at<uchar>(v2 + 1,	u2 - 1,	0) = 255;
			img.at<uchar>(v2 + 1,	u2,	0) = 255;
			img.at<uchar>(v2 + 1,	u2 + 1,	0) = 255;
			cv::imshow(full_code, img);
		}
	}
}

int main(int argc, char** argv) {
	std::string data_path = ros::package::getPath("roadprintz_camera_calibration");
	boost::filesystem::path image_folder_path = boost::filesystem::path(data_path + "/white_tape_data/images/");
	std::ofstream roi_output_file;
	roi_output_file.open(data_path + "/white_tape_data/rois.csv");
	
	for(boost::filesystem::directory_entry& x : boost::filesystem::directory_iterator(image_folder_path)){
		img_pure = cv::imread(x.path().native());
		img_pure.copyTo(img);
		
		printf("%s\n", x.path().filename().native().c_str());
		
		for(int i = 0; i < 6; i++){
			for(int c = 0; c < 4; c++){
				std::string code;
				switch(c){
					case 0:
						code = "top left";
						break;
					case 1:
						code = "top right";
						break;
					case 2:
						code = "bottom left";
						break;
					case 3:
						code = "bottom right";
						break;
				}
				full_code = "Tape " + std::to_string(i + 1) + " " + code;
				
				printf("\t%s:\n", full_code.c_str());
				
				cc = 0;
				
				cv::namedWindow(full_code);
				cv::setMouseCallback(full_code, click_cb);
				cv::imshow(full_code, img);
				
				cv::waitKey();
				
				if(cc == 2){
					roi_output_file << x.path().filename().native() << ", " << i << ", " << c << ", "
						 << u1 << ", " << v1 << ", " << u2 << ", " << v2 << "\n";
				}
				
				cv::destroyAllWindows();
			}
		}
	}

	roi_output_file.close();
	return 0;
}
