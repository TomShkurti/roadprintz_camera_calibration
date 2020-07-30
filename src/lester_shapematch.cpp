#include <iostream>

#include <ros/package.h>

#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

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
	
	for(boost::filesystem::directory_entry& x : boost::filesystem::directory_iterator(image_folder_path)){
		cv::Mat img = cv::imread(x.path().native());
		
		printf("%s\n", x.path().filename().native().c_str());
		
		/*cv::namedWindow("Raw");
		cv::imshow("Raw", img);*/

		
		//We want YELLOW things!
		cv::Mat bgr_chan[3];
		cv::split(img,bgr_chan);//Why BGR and not RGB?? Weird.
		/*cv::namedWindow("Chan 1");
		cv::imshow("Chan 1", bgr_chan[0]);
		cv::namedWindow("Chan 2");
		cv::imshow("Chan 2", bgr_chan[1]);
		cv::namedWindow("Chan 3");
		cv::imshow("Chan 3", bgr_chan[2]);*/
		cv::Mat hsv;
		cv::cvtColor(img,hsv,CV_BGR2HSV);
		cv::Mat hsv_chan[3];
		cv::split(hsv, hsv_chan);
		/*cv::namedWindow("HSV Chan 1");
		cv::imshow("HSV Chan 1", hsv_chan[0]);
		cv::namedWindow("HSV Chan 2");
		cv::imshow("HSV Chan 2", hsv_chan[1]);
		cv::namedWindow("HSV Chan 3");
		cv::imshow("HSV Chan 3", hsv_chan[2]);
		cv::waitKey();*/
		
		cv::Mat gr;
		cv::multiply(bgr_chan[1], bgr_chan[2], gr, 1.0 / 255);
		cv::Mat grv;
		cv::multiply(hsv_chan[2], gr, grv, 1.0 / 255);
		cv::Mat yellow;
		cv::multiply(grv, 255 - bgr_chan[0], yellow, 1.0 / 255);
		/*cv::namedWindow("Yellow");
		cv::imshow("Yellow", yellow);
		cv::waitKey();*/
		
		//Threshold the shape
		//TODO This thresholding is extremely rigid and may need to be
		//tweaked with each acquisition run. An adaptive thresholding
		//method would be a huge improvement.
		cv::Mat thresh = yellow > 50;
		/*cv::imshow("Threshold", thresh);
		cv::waitKey();*/
		
		//Choose the criteria we will compare shapes against.
		//Currently set up to accomodate differences between the right
		//and left target even though there are none at the moment.
		//TODO: 
		double criteria [7];
		if(x.path().filename().native().find("left") == std::string::npos){
			//Right thresholds
			criteria[0] = 0.7;//Greater than 0
			criteria[1] = 0.2;//Greater than 1
			criteria[2] = 0.275;//Greater than 2
			criteria[3] = 0.03;//Greater than 3
			criteria[4] = 0.5e-3;//Abs greater than 4
			criteria[5] = 0.0;//Don't use 5
			criteria[6] = 0.1;//less than 6
		} else {
			//Left thresholds
			criteria[0] = 0.7;//Greater than 0
			criteria[1] = 0.2;//Greater than 1
			criteria[2] = 0.275;//Greater than 2
			criteria[3] = 0.03;//Greater than 3
			criteria[4] = 0.5e-3;//Abs greater than 4
			criteria[5] = 0.0;//Don't use 5
			criteria[6] = 0.1;//less than 6
		}
		
		//Split the image into multiple contiguous sections
		
		//Stuff the point cloud representation
		pcl::PointCloud<pcl::PointXYZ> * cloud = new pcl::PointCloud<pcl::PointXYZ>();
		//Has to be PointXYZ even though 2D because
		//PCL does not have full functionality for 2D point clouds.
		for(int y = 0; y < thresh.rows; y++){
			for(int x = 0; x < thresh.cols; x++){
				if(thresh.at<bool>(y * thresh.cols + x)){
					pcl::PointXYZ p;
					p.x = x;
					p.y = y;
					p.z = 0;
					cloud->points.push_back(p);
				}
			}
		}
		
		//Cluster the points
		std::vector<pcl::PointIndices> inds;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(1.5);
		ec.setMinClusterSize(50);//Anything smaller than this is not worth our time.
		ec.setMaxClusterSize(10000000);//TODO Can this just be left blank?
		ec.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud));
		ec.extract(inds);
		
		//For each cluster...
		//cv::namedWindow("Shape candidate");
		cv::Mat the_chosen_one;
		int chose = 0;
		for(int i = 0; i < inds.size(); i++){
			//Take the Hu moments.
			cv::Mat thresh_masked = cv::Mat(thresh.rows, thresh.cols, CV_8UC1, cvScalar(0));;
			for(int p = 0; p < inds[i].indices.size(); p++){
				pcl::PointXYZ pnt = cloud->points[inds[i].indices[p]];
				thresh_masked.at<uchar>(pnt.y, pnt.x) = 255;
			}
			
			cv::Moments mo = moments(thresh_masked, true);
			double hm [7];
			cv::HuMoments(mo, hm);
			//Output used to generate CSV files for shape detection.
			//TODO Either automate this or remove it entirely.
			/*printf("\t");
			cv::imshow("Shape candidate", thresh_masked);
			for(int m = 0; m < 7; m++){
				printf("%f,\t", hm[m]);
			}
			printf("\n");
			printf("\t%d %d %d %d %d %d %d\n",
				hm[0] > criteria[0],
				hm[1] > criteria[1],
				hm[2] > criteria[2],
				hm[3] > criteria[3],
				abs(hm[4]) > criteria[4],
				true,
				hm[6] < criteria[6]
			);
			cv::waitKey();*/
			if(
				hm[0] > criteria[0] &&
				hm[1] > criteria[1] &&
				hm[2] > criteria[2] &&
				hm[3] > criteria[3] &&
				abs(hm[4]) > criteria[4] &&
				//Don't use 5
				hm[6] < criteria[6]
			){
				chose ++;
				the_chosen_one = thresh_masked;
			}
		}
		if(chose == 0){
			printf("\t\e[31mFailed to find a shape satisfying geometric constraints.\e[39m\n");
			continue;
		} if(chose > 1){
			printf("\t\e[31mFound more than one shape satisfying geometric constraints.\e[39m\n");
			continue;
		}
		/*cv::namedWindow("Chosen Shape");
		cv::imshow("Chosen Shape", the_chosen_one);
		cv::waitKey();*/
		
		//Re-window ONLY the shape we found and a little of its surroundings.
		cv::Mat dilated;
		cv::dilate(the_chosen_one, dilated, cv::Mat(), cv::Point(-1,-1), 2);
		/*cv::namedWindow("Dilated Outline");
		cv::imshow("Dilated Outline", dilated);
		cv::waitKey();*/
		
		cv::Mat masked = dilated & yellow;
		/*cv::namedWindow("Target Window");
		cv::imshow("Target Window", masked);
		cv::waitKey();*/
		
		
		//Ow the Edge
		cv::Mat edges;
		cv::Canny( masked, edges, 75, 200, 3 );
		/*cv::namedWindow("Edges");
		cv::imshow("Edges", edges);
		cv::waitKey();*/
		
		//Find lines
		std::vector<cv::Vec4i> lines;
		cv::HoughLinesP(edges, lines, 1, CV_PI/45, 25, 25, 10);
		cv::Mat lines_show = img.clone();
		for(int i = 0; i < lines.size(); i++){
			cv::Vec4i l = lines[i];
        		line(lines_show, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, cv::LINE_AA);
		}
		/*cv::namedWindow("Found Lines");
		cv::imshow("Found Lines", lines_show);
		cv::waitKey();*/
		
		//Pair the lines up
		std::vector<cv::Point2d> intersections;
		double x_max = 0.0;
		double x_min = 10000.0;
		double y_max = 0.0;
		double y_min = 10000.0;
		for(int i = 0; i < lines.size(); i++){
			cv::Vec4i l_1 = lines[i];
			//Check extrema
			if(l_1[0] > x_max){
				x_max = l_1[0];
			} if(l_1[0] < x_min){
				x_min = l_1[0];
			}
			if(l_1[2] > x_max){
				x_max = l_1[2];
			} if(l_1[2] < x_min){
				x_min = l_1[2];
			}
			
			
			if(l_1[1] > y_max){
				y_max = l_1[1];
			} if(l_1[1] < y_min){
				y_min = l_1[1];
			}
			if(l_1[3] > y_max){
				y_max = l_1[3];
			} if(l_1[3] < y_min){
				y_min = l_1[3];
			}
			
			double angle_1 = atan2(abs(l_1[0] - l_1[2]), abs(l_1[1] - l_1[3]));
			
			for(int j = i + 1; j < lines.size(); j++){
				//Check the angle they make with each other...
				cv::Vec4i l_2 = lines[j];
				double angle_2 = atan2(abs(l_2[0] - l_2[2]), abs(l_2[1] - l_2[3]));
				
				/*printf("\tAngle is %f\n", abs(angle_1 - angle_2));
				cv::Mat lines_show = img.clone();
				line(lines_show, cv::Point(l_1[0], l_1[1]), cv::Point(l_1[2], l_1[3]), cv::Scalar(0,0,255), 1, cv::LINE_AA);
				line(lines_show, cv::Point(l_2[0], l_2[1]), cv::Point(l_2[2], l_2[3]), cv::Scalar(255,0,0), 1, cv::LINE_AA);*/
				
				//And if they are nearly perpendicular...
				if(abs(angle_1 - angle_2) > 0.1){
					cv::Point2d o1 = cv::Point(l_1[0], l_1[1]);
					cv::Point2d p1 = cv::Point(l_1[2], l_1[3]);
					cv::Point2d o2 = cv::Point(l_2[0], l_2[1]);
					cv::Point2d p2 = cv::Point(l_2[2], l_2[3]);
					
					cv::Point2d x = o2 - o1;
					cv::Point2d d1 = p1 - o1;
					cv::Point2d d2 = p2 - o2;

					double cross = d1.x*d2.y - d1.y*d2.x;

					double t1 = (x.x * d2.y - x.y * d2.x)/cross;
					cv::Point2f r = o1 + d1 * t1;
					
					intersections.push_back(r);
					
					/*lines_show.at<cv::Vec3b>((int)std::rint(r.y) - 1, (int)std::rint(r.x) - 1)[1] = 255;
					lines_show.at<cv::Vec3b>((int)std::rint(r.y) + 1, (int)std::rint(r.x) - 1)[1] = 255;
					lines_show.at<cv::Vec3b>((int)std::rint(r.y) - 1, (int)std::rint(r.x) + 1)[1] = 255;
					lines_show.at<cv::Vec3b>((int)std::rint(r.y) + 1, (int)std::rint(r.x) + 1)[1] = 255;
					lines_show.at<cv::Vec3b>((int)std::rint(r.y), (int)std::rint(r.x))[1] = 255;*/
				}
				/*cv::namedWindow("Found Lines");
				cv::imshow("Found Lines", lines_show);
				cv::waitKey();*/
			}
		}
		
		if(intersections.size() < 3){
			printf("\t\e[31mFailed to find enough crossing points.\e[39m\n");
			continue;
		}
		
		cv::Point2d centroid = cv::Point2d((x_min + x_max) / 2.0, (y_min + y_max) / 2.0);
		
		/*cv::Mat points_show = img.clone();
		points_show.at<cv::Vec3b>((int)std::rint(centroid.y) - 1, (int)std::rint(centroid.x) - 1)[2] = 255;
		points_show.at<cv::Vec3b>((int)std::rint(centroid.y) + 1, (int)std::rint(centroid.x) - 1)[2] = 255;
		points_show.at<cv::Vec3b>((int)std::rint(centroid.y) - 1, (int)std::rint(centroid.x) + 1)[2] = 255;
		points_show.at<cv::Vec3b>((int)std::rint(centroid.y) + 1, (int)std::rint(centroid.x) + 1)[2] = 255;
		points_show.at<cv::Vec3b>((int)std::rint(centroid.y), (int)std::rint(centroid.x))[2] = 255;
		for(int i = 0; i < intersections.size(); i++){
			points_show.at<cv::Vec3b>((int)std::rint(intersections[i].y) - 1, (int)std::rint(intersections[i].x) - 1)[1] = 255;
			points_show.at<cv::Vec3b>((int)std::rint(intersections[i].y) + 1, (int)std::rint(intersections[i].x) - 1)[1] = 255;
			points_show.at<cv::Vec3b>((int)std::rint(intersections[i].y) - 1, (int)std::rint(intersections[i].x) + 1)[1] = 255;
			points_show.at<cv::Vec3b>((int)std::rint(intersections[i].y) + 1, (int)std::rint(intersections[i].x) + 1)[1] = 255;
			points_show.at<cv::Vec3b>((int)std::rint(intersections[i].y), (int)std::rint(intersections[i].x))[1] = 255;
		}
		cv::namedWindow("Found Intersections");
		cv::imshow("Found Intersections", points_show);
		cv::waitKey();*/
		
		double min_dist = 10000.0;
		int min_ind = -1;
		for(int i = 0; i < intersections.size(); i++){
			if(cv::norm(intersections[i] - centroid) < min_dist){
				min_dist = cv::norm(intersections[i] - centroid);
				min_ind = i;
			}
		}
		
		cv::Point2d goal_point = intersections[min_ind];
		
		cv::Mat final_show = img.clone();
		final_show.at<cv::Vec3b>((int)std::rint(goal_point.y) - 1, (int)std::rint(goal_point.x) - 1)[2] = 255;
		final_show.at<cv::Vec3b>((int)std::rint(goal_point.y) + 1, (int)std::rint(goal_point.x) - 1)[2] = 255;
		final_show.at<cv::Vec3b>((int)std::rint(goal_point.y) - 1, (int)std::rint(goal_point.x) + 1)[2] = 255;
		final_show.at<cv::Vec3b>((int)std::rint(goal_point.y) + 1, (int)std::rint(goal_point.x) + 1)[2] = 255;
		final_show.at<cv::Vec3b>((int)std::rint(goal_point.y), (int)std::rint(goal_point.x))[2] = 255;
		cv::namedWindow("Detected Point");
		cv::imshow("Detected Point", final_show);
		cv::waitKey(1000);
		
		printf("\tDetected point (%f, %f)\n", goal_point.x, goal_point.y);
		
		cv::destroyAllWindows();
	}
	
	return 0;
}
