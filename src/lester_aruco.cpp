#include <iostream>

#include <ros/package.h>

#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>

//TODO Are all of these ACTUALLY necessary?
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

int main(int argc, char** argv) {

	std::string data_path = ros::package::getPath("roadprintz_camera_calibration");
	boost::filesystem::path image_folder_path = boost::filesystem::path(data_path + "/lester_base_calib_data/images/");
	
	//Grab the positions and store them for later use
	std::vector<std::vector<double> > left_positions;
	std::ifstream left_positions_in(data_path + "/lester_base_calib_data/arm_positions_left.csv");
	if(!left_positions_in.is_open()){
		printf(
			"\e[31mCould not find \"%s\".\e[39m\n",
			(data_path + "/lester_base_calib_data/arm_positions_left.csv").c_str()
		);
		return 0;
	}
	std::string l_tmp;
	while(getline(left_positions_in, l_tmp)){
		std::vector<double> la(16);
		const char * l_tmp_c = l_tmp.c_str();
		sscanf(
			l_tmp_c,
			"%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, ",
			&la[0],		&la[1],		&la[2],		&la[3],
			&la[4],		&la[5],		&la[6],		&la[7],
			&la[8],		&la[9],		&la[10],	&la[11],
			&la[12],	&la[13],	&la[14],	&la[15]
		);
		left_positions.push_back(la);
	}
	left_positions_in.close();
			
	std::vector<std::vector<double> > right_positions;
	std::ifstream right_positions_in(data_path + "/lester_base_calib_data/arm_positions_right.csv");
	if(!right_positions_in.is_open()){
		printf(
			"\e[31mCould not find \"%s\".\e[39m\n",
			(data_path + "/lester_base_calib_data/arm_positions_right.csv").c_str()
		);
		return 0;
	}
	std::string r_tmp;
	while(getline(right_positions_in, r_tmp)){
		std::vector<double> ra(16);
		const char * r_tmp_c = r_tmp.c_str();
		sscanf(
			r_tmp_c,
			"%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, ",
			&ra[0],		&ra[1],		&ra[2],		&ra[3],
			&ra[4],		&ra[5],		&ra[6],		&ra[7],
			&ra[8],		&ra[9],		&ra[10],	&ra[11],
			&ra[12],	&ra[13],	&ra[14],	&ra[15]
		);
		right_positions.push_back(ra);
	}
	right_positions_in.close();
	
	std::ofstream output_file;
	output_file.open(data_path + "/lester_base_calib_data/detections.csv");
	
	cv::Ptr<cv::aruco::DetectorParameters> aruco_params = cv::aruco::DetectorParameters::create();
	//TODO investigate why subpix detection is not working. OCV version issue??
	//aruco_params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
	cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
	
	//ID images
	for(boost::filesystem::directory_entry& x : boost::filesystem::directory_iterator(image_folder_path)){
		cv::Mat img = cv::imread(x.path().native());
		
		printf("%s\n", x.path().filename().native().c_str());
		
		std::vector<int> aruco_ids;
		std::vector<std::vector<cv::Point2f>> aruco_corners, aruco_rejects;
		cv::aruco::detectMarkers(img, aruco_dict, aruco_corners, aruco_ids, aruco_params, aruco_rejects);
		
		//TODO Add more aggressive detection here- right # of markers, right ID, etc.
		if(aruco_corners.size() > 0){
			printf("Successfully detected marker.\n");
			cv::Mat final_show = img.clone();
			//for(int i = 0; i < aruco_corners.size(); i++){
			//	for(int j = 0; j < aruco_corners[i].size(); j++){
					double u = aruco_corners[0][1].x;
					double v = aruco_corners[0][1].y;
					
					final_show.at<cv::Vec3b>((int)std::rint(v) - 1, (int)std::rint(u) - 1) = {255, 0, 0};
					final_show.at<cv::Vec3b>((int)std::rint(v) + 1, (int)std::rint(u) - 1) = {255, 0, 0};
					final_show.at<cv::Vec3b>((int)std::rint(v) - 1, (int)std::rint(u) + 1) = {255, 0, 0};
					final_show.at<cv::Vec3b>((int)std::rint(v) + 1, (int)std::rint(u) + 1) = {255, 0, 0};
					final_show.at<cv::Vec3b>((int)std::rint(v) - 2, (int)std::rint(u) - 2) = {255, 0, 0};
					final_show.at<cv::Vec3b>((int)std::rint(v) + 2, (int)std::rint(u) - 2) = {255, 0, 0};
					final_show.at<cv::Vec3b>((int)std::rint(v) - 2, (int)std::rint(u) + 2) = {255, 0, 0};
					final_show.at<cv::Vec3b>((int)std::rint(v) + 2, (int)std::rint(u) + 2) = {255, 0, 0};
					final_show.at<cv::Vec3b>((int)std::rint(v), (int)std::rint(u)) = {255, 0, 0};
				//}
			//}
			
			printf("\tDetected point (%f, %f)\n", u, v);
			
			cv::namedWindow("Detected Point");
			cv::imshow("Detected Point", final_show);
			cv::waitKey(500);
		
			cv::destroyAllWindows();
			
			
			//Write our data.
			std::string fname = x.path().filename().native();
			std::string number_string = fname.substr(
				fname.find_first_of("t") + 1,
				fname.find_first_of(".") - fname.find_first_of("t") - 1
			);
			int index = std::stoi(number_string);
		
			std::vector<double> position_line;
			if(x.path().filename().native().find("left") == std::string::npos){
				//Right lookup
				position_line = right_positions[index];
				output_file << "0, ";
			} else {
				//Left lookup
				position_line = left_positions[index];
				output_file << "1, ";
			}
			for(int i = 0; i < 16; i++){
				output_file << std::to_string(position_line[i]) << ", ";
			}
			output_file << std::to_string(u) << ", ";
			output_file << std::to_string(v) << "\n";
		}
	}
	output_file.close();
	
	return 0;
}
