#include <iostream>

#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <template_matching_py/TemplateMatchingSrv.h>
#include <hmi_set_template_align/SetTemplateAlignment.h>

#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv) {

	std::string data_path = ros::package::getPath("roadprintz_camera_calibration");
	std::string image_folder_path = data_path + "/white_tape_data/images/";
	
	std::ifstream roi_stream(data_path + "/white_tape_data/rois.csv");
	std::ifstream poses_stream(data_path + "/white_tape_data/arm_positions.csv");
	
	std::ofstream output(data_path + "/white_tape_data/detections.csv");
	
	//char buffer[32];
	std::string data_string;
	while(!roi_stream.eof()){
		//Extract the detection data.
		std::getline(roi_stream, data_string);
		
		int img_num;
		int tape_num;
		int corner_num;
		
		int u1;
		int v1;
		int u2;
		int v2;
		
		sscanf(
			data_string.c_str(),
			"img%d.png, %d, %d, %d, %d, %d, %d",
			&img_num, &tape_num, &corner_num,
			&u1, &v1, &u2, &v2
		);
		
		//Split out the corner from the rest of the image
		cv::Mat img = cv::imread(image_folder_path + "img" + std::to_string(img_num) + ".png");
		cv::Mat roi = img(cv::Rect(std::min(u1, u2), std::min(v1, v2), abs(u2 - u1), abs(v2 - v1)));
		//cv::namedWindow("roi");
		//cv::imshow("roi", roi);
		//cv::waitKey();
		//cv::destroyAllWindows();
		
		//Convert to whiteness
		cv::Mat hsv;
		cv::cvtColor(roi,hsv,CV_BGR2HSV);
		cv::Mat hsv_chan[3];
		cv::split(hsv, hsv_chan);
		/*cv::namedWindow("HSV Chan 1");
		cv::imshow("HSV Chan 1", hsv_chan[0]);
		cv::namedWindow("HSV Chan 2");
		cv::imshow("HSV Chan 2", hsv_chan[1]);
		cv::namedWindow("HSV Chan 3");
		cv::imshow("HSV Chan 3", hsv_chan[2]);
		cv::waitKey();*/
		cv::Mat intensity = hsv_chan[2];
		
		//Run Harris corner detection.
		cv::Mat corners;
		cv::cornerHarris(intensity, corners, 2, 3, 0.04);
		cv::normalize(corners, corners, 1, 0, cv::NORM_MINMAX);
		/*cv::namedWindow("Cornerness");
		cv::imshow("Cornerness", corners);
		cv::waitKey();
		cv::destroyAllWindows();*/
		
		double max;
		double min;
		cv::minMaxLoc(corners, &min, &max);
		cv::Mat c_thresh = corners > 0.95 * max;
		cv::namedWindow("Cornerness Threshold");
		cv::imshow("Cornerness Threshold", c_thresh);
		cv::waitKey(500);
		cv::destroyAllWindows();
		
		//Centroidize the corner
		int c_u = 0;
		int c_v = 0;
		int c_c = 0;
		int max_u = -1;
		int max_v = -1;
		int min_u = 9000;
		int min_v = 9000;
		for(int v = 0; v < c_thresh.rows; v++){
			for(int u = 0; u < c_thresh.cols; u++){
				if(c_thresh.at<bool>(v * c_thresh.cols + u)){
					c_u += u;
					c_v += v;
					c_c ++;
					
					if(u > max_u){
						max_u = u;
					}
					if(u < min_u){
						min_u = u;
					}
					if(v > max_v){
						max_v = v;
					}
					if(v < min_v){
						min_v = v;
					}
				}
			}
		}
		if(max_u - min_u > 5 || max_v - min_v > 5){
			printf("\t\e[31mRejected point due to spread being too large.\e[39m\n");
			continue;
		}
		
		double u_corner = (double)c_u / (double)c_c;
		double v_corner = (double)c_v / (double)c_c;
		if(
			u_corner < 3.0 || u_corner > c_thresh.cols - 3
			||
			v_corner < 3.0 || v_corner > c_thresh.rows - 3
		){
			printf("\t\e[31mRejected point due to being too close to the edge.\e[39m\n");
			continue;
		}
		
		printf("\tAccepted point.\n");
		
		//Write point data
		u_corner += std::min(u1, u2);
		v_corner += std::min(v1, v2);
		std::string pose_string;
		for(int i = 0; i < img_num; i++){
			std::getline(poses_stream, pose_string);
		}
		poses_stream.clear();
		poses_stream.seekg(0);
		
		output << pose_string << tape_num << ", " << corner_num <<
			", " << u_corner << ", " << v_corner << "\n";
	}
	
	output.close();
	
	return 0;
}
